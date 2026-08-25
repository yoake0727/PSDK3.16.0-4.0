#include "h30t_rtsp_publisher.hpp"

#include "h30t_config.hpp"

#if !defined(H30T_STREAM_PIPELINE_STUB)
#include "dji_logger.h"
#endif

#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/error.h>
}
#endif

class H30tRtspPublisher::Impl {
public:
#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
    Impl() : context(NULL), stream(NULL), header_written(false) {}
    AVFormatContext *context;
    AVStream *stream;
    bool header_written;

    static std::string ErrorText(int error_code)
    {
        char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
        if (av_strerror(error_code, buffer, sizeof(buffer)) < 0) {
            return std::string("FFmpeg error ") + std::to_string(error_code);
        }
        return buffer;
    }
#else
    Impl() : open(false) {}
    bool open;
#endif
};

H30tRtspPublisher::H30tRtspPublisher() : impl_(new Impl) {}
H30tRtspPublisher::~H30tRtspPublisher() { Close(); }

bool H30tRtspPublisher::Open(const H30tStreamPipelineConfig &config, int width, int height)
{
    Close();
#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
    const int context_result = avformat_alloc_output_context2(
        &impl_->context, NULL, "rtsp", config.rtsp_url.c_str());
    if (context_result < 0 || impl_->context == NULL) {
        return false;
    }
    impl_->stream = avformat_new_stream(impl_->context, NULL);
    if (impl_->stream == NULL) {
        Close(); return false;
    }
    impl_->stream->time_base = AVRational{1, 90000};
    impl_->stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    impl_->stream->codecpar->codec_id = AV_CODEC_ID_H264;
    impl_->stream->codecpar->codec_tag = 0;
    impl_->stream->codecpar->width = width;
    impl_->stream->codecpar->height = height;
    AVDictionary *options = NULL;
    av_dict_set(&options, "rtsp_transport", config.rtsp_transport.c_str(), 0);
    av_dict_set(&options, "stimeout", "2000000", 0);
    av_dict_set(&options, "rw_timeout", "2000000", 0);
    const int result = avformat_write_header(impl_->context, &options);
    av_dict_free(&options);
    if (result < 0) {
        USER_LOG_ERROR("Open RTSP publisher %s failed: %s (%d)",
                       h30t_config::RedactRtspUrl(config.rtsp_url).c_str(),
                       Impl::ErrorText(result).c_str(), result);
        Close();
        return false;
    }
    impl_->header_written = true;
    return true;
#else
    (void)config; (void)width; (void)height;
    return false;
#endif
}

bool H30tRtspPublisher::Write(const std::uint8_t *data, int size,
                              std::int64_t timestamp, bool keyframe)
{
#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
    if (!IsOpen() || data == NULL || size <= 0) return false;
    AVPacket packet;
    av_init_packet(&packet);
    packet.data = const_cast<std::uint8_t *>(data);
    packet.size = size;
    packet.stream_index = impl_->stream->index;
    packet.pts = timestamp;
    packet.dts = timestamp;
    if (keyframe) packet.flags |= AV_PKT_FLAG_KEY;
    const int result = av_interleaved_write_frame(impl_->context, &packet);
    if (result < 0) {
        USER_LOG_ERROR("Write RTSP H.264 packet failed: %s (%d)",
                       Impl::ErrorText(result).c_str(), result);
        return false;
    }
    return true;
#else
    (void)data; (void)size; (void)timestamp; (void)keyframe;
    return false;
#endif
}

bool H30tRtspPublisher::IsOpen() const
{
#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
    return impl_->context != NULL && impl_->stream != NULL;
#else
    return false;
#endif
}

void H30tRtspPublisher::Close()
{
#if !defined(H30T_STREAM_PIPELINE_STUB) && defined(FFMPEG_INSTALLED)
    if (impl_->context != NULL) {
        if (impl_->header_written) av_write_trailer(impl_->context);
        if (!(impl_->context->oformat->flags & AVFMT_NOFILE) && impl_->context->pb != NULL) {
            avio_closep(&impl_->context->pb);
        }
        avformat_free_context(impl_->context);
    }
    impl_->context = NULL;
    impl_->stream = NULL;
    impl_->header_written = false;
#endif
}
