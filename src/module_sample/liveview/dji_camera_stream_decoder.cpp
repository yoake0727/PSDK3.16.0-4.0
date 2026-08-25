/**
 ********************************************************************
 * @file    dji_camera_stream_decoder.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dji_camera_stream_decoder.hpp"
#include "unistd.h"
#include "pthread.h"
#include "dji_logger.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
DJICameraStreamDecoder::DJICameraStreamDecoder()
    : initSuccess(false),
      cbThreadIsRunning(false),
      cbThreadStatus(-1),
      cb(nullptr),
      cbUserParam(nullptr),
#ifdef FFMPEG_INSTALLED
      pCodecCtx(nullptr),
      pCodec(nullptr),
      pCodecParserCtx(nullptr),
      pSwsCtx(nullptr),
      pFrameYUV(nullptr),
      pFrameRGB(nullptr),
      decodedWidth(0),
      decodedHeight(0),
      decodedPixelFormat(AV_PIX_FMT_NONE),
#endif
      rgbBuf(nullptr),
      bufSize(0)
{
    pthread_mutex_init(&decodemutex, nullptr);
}

DJICameraStreamDecoder::~DJICameraStreamDecoder()
{
    registerCallback(nullptr, nullptr);
    cleanup();
    pthread_mutex_destroy(&decodemutex);
}

bool DJICameraStreamDecoder::init()
{
    pthread_mutex_lock(&decodemutex);

    if (true == initSuccess) {
        USER_LOG_INFO("Decoder already initialized.\n");
        pthread_mutex_unlock(&decodemutex);
        return true;
    }

#ifdef FFMPEG_INSTALLED
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(nullptr);
    if (!pCodecCtx) {
        goto init_failed;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        goto init_failed;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx) {
        goto init_failed;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV) {
        goto init_failed;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
        goto init_failed;
    }

    pSwsCtx = nullptr;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
#endif
    initSuccess = true;
    pthread_mutex_unlock(&decodemutex);

    return true;

init_failed:
    cleanupUnlocked();
    pthread_mutex_unlock(&decodemutex);
    return false;
}

void DJICameraStreamDecoder::cleanup()
{
    pthread_mutex_lock(&decodemutex);
    cleanupUnlocked();
    pthread_mutex_unlock(&decodemutex);
}

void DJICameraStreamDecoder::cleanupUnlocked()
{
    initSuccess = false;

#ifdef FFMPEG_INSTALLED
    if (nullptr != pSwsCtx) {
        sws_freeContext(pSwsCtx);
        pSwsCtx = nullptr;
    }

    if (nullptr != pFrameYUV) {
        av_frame_free(&pFrameYUV);
    }

    if (nullptr != pCodecParserCtx) {
        av_parser_close(pCodecParserCtx);
        pCodecParserCtx = nullptr;
    }

    if (nullptr != pCodecCtx) {
        avcodec_free_context(&pCodecCtx);
    }
    pCodec = nullptr;

    if (nullptr != rgbBuf) {
        av_free(rgbBuf);
        rgbBuf = nullptr;
    }

    if (nullptr != pFrameRGB) {
        av_frame_free(&pFrameRGB);
    }

    decodedWidth = 0;
    decodedHeight = 0;
    decodedPixelFormat = AV_PIX_FMT_NONE;
#endif
    bufSize = 0;
}

void *DJICameraStreamDecoder::callbackThreadEntry(void *p)
{
    //DSTATUS_PRIVATE("****** Decoder Callback Thread Start ******\n");
    usleep(50 * 1000);
    static_cast<DJICameraStreamDecoder *>(p)->callbackThreadFunc();
    return nullptr;
}

void DJICameraStreamDecoder::callbackThreadFunc()
{
    while (cbThreadIsRunning) {
        CameraRGBImage copyOfImage;
        if (!decodedImageHandler.getNewImageWithLock(copyOfImage, 1000)) {
            //DDEBUG_PRIVATE("Decoder Callback Thread: Get image time out\n");
            continue;
        }

        if (cb) {
            (*cb)(copyOfImage, cbUserParam);
        }
    }
}

void DJICameraStreamDecoder::decodeBuffer(const uint8_t *buf, int bufLen)
{
    if (buf == nullptr || bufLen <= 0) {
        return;
    }

    const uint8_t *pData = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

#ifdef FFMPEG_INSTALLED
    AVPacket pkt;
    av_init_packet(&pkt);
    pthread_mutex_lock(&decodemutex);
    if (!initSuccess || !pCodecParserCtx || !pCodecCtx || !pFrameYUV || !pFrameRGB) {
        pthread_mutex_unlock(&decodemutex);
        av_free_packet(&pkt);
        return;
    }

    while (remainingLen > 0) {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        if (processedLen <= 0) {
            USER_LOG_WARN("H264 parser made no progress, processed length: %d", processedLen);
            break;
        }

        remainingLen -= processedLen;
        pData += processedLen;

        if (pkt.size > 0) {
            int gotPicture = 0;
            const int decodeResult = avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);
            if (decodeResult < 0) {
                continue;
            }

            if (!gotPicture) {
                ////DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            } else {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                AVPixelFormat pixelFormat = static_cast<AVPixelFormat>(pFrameYUV->format);
                ////DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if (w <= 0 || h <= 0 || pixelFormat == AV_PIX_FMT_NONE) {
                    continue;
                }

                const bool frameFormatChanged = nullptr == pSwsCtx || nullptr == rgbBuf
                                                || decodedWidth != w || decodedHeight != h
                                                || decodedPixelFormat != pixelFormat;
                if (frameFormatChanged) {
                    pSwsCtx = sws_getCachedContext(pSwsCtx,
                                                   w, h, pixelFormat,
                                                   w, h, AV_PIX_FMT_RGB24,
                                                   SWS_BILINEAR, nullptr, nullptr, nullptr);
                    if (nullptr == pSwsCtx) {
                        continue;
                    }

                    const int newBufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    if (newBufSize <= 0) {
                        continue;
                    }

                    uint8_t *newRgbBuf = static_cast<uint8_t *>(av_malloc(static_cast<size_t>(newBufSize)));
                    if (nullptr == newRgbBuf) {
                        continue;
                    }

                    if (avpicture_fill(reinterpret_cast<AVPicture *>(pFrameRGB), newRgbBuf,
                                       AV_PIX_FMT_RGB24, w, h) < 0) {
                        av_free(newRgbBuf);
                        continue;
                    }

                    if (nullptr != rgbBuf) {
                        av_free(rgbBuf);
                    }
                    rgbBuf = newRgbBuf;
                    bufSize = static_cast<size_t>(newBufSize);
                    decodedWidth = w;
                    decodedHeight = h;
                    decodedPixelFormat = pixelFormat;
                }

                if (nullptr != pSwsCtx && nullptr != rgbBuf) {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;

                    decodedImageHandler.writeNewImageWithLock(pFrameRGB->data[0], static_cast<int>(bufSize), w, h);
                }
            }
        }
    }
    pthread_mutex_unlock(&decodemutex);
    av_free_packet(&pkt);
#endif
}

bool DJICameraStreamDecoder::registerCallback(CameraImageCallback f, void *param)
{
    cb = f;
    cbUserParam = param;

    /* When users register a non-nullptr callback, we will start the callback thread. */
    if (nullptr != cb) {
        if (!cbThreadIsRunning) {
            cbThreadStatus = pthread_create(&callbackThread, nullptr, callbackThreadEntry, this);
            if (0 == cbThreadStatus) {
                //DSTATUS_PRIVATE("User callback thread created successfully!\n");
                cbThreadIsRunning = true;
                return true;
            } else {
                //DERROR_PRIVATE("User called thread creation failed!\n");
                cbThreadIsRunning = false;
                return false;
            }
        } else {
            //DERROR_PRIVATE("Callback thread already running!\n");
            return true;
        }
    } else {
        if (cbThreadStatus == 0) {
            cbThreadIsRunning = false;
            pthread_join(callbackThread, nullptr);
            cbThreadStatus = -1;
        }
        return true;
    }
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
