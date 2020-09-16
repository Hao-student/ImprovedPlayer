#include <iostream>
#include <queue>
#include <thread>
#include <chrono>
//openAL库
#include "al.h"
#include "alc.h"
//FFmpeg库，SDL库
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libswresample/swresample.h>
#include <SDL2/SDL.h>
}

#define __STDC_CONSTANT_MACROS
#define SDL_MAIN_HANDLED
#define MAX_AUDIO_FRAME_SIZE 48000 * 4 
#define NUMBUFFERS 4

using namespace std;

//声明结构体，用于存储解码音频帧相关数据
typedef struct audioFrame {
	uint8_t* data;
	int data_size;
	int sample_rate;
	double audio_clock;
}AUDIOFRAME, * AudioFrame;

//刷新事件
#define REFRESH_EVENT (SDL_USEREVENT+1)
//break事件
#define BREAK_EVENT (SDL_USEREVENT+2)

std::queue<AudioFrame> audioFrameQueue; //存储解码后音频帧
ALuint source;//openAL源
double audio_pts;//音频时间戳
int window_w, window_h;//SDL窗口宽高
int thread_exit = 0;//用于判定是否退出
bool thread_pause = false;//用于判定是否暂停
bool forward_state = false;//用于判定是否快进
double video_forward = 0;//快进量

//openAL填充数据
int feedAudioData(ALuint& bufferID)
{
	if (audioFrameQueue.empty())//若队列为空则返回-1
		return -1;
	AudioFrame frame = audioFrameQueue.front();//读取音频帧
	audioFrameQueue.pop();//读取后弹出
	if (frame == nullptr)
		return -1;
	//把数据写入buffer
	alBufferData(bufferID, AL_FORMAT_STEREO16, frame->data, frame->data_size, frame->sample_rate);
	//将buffer放回缓冲区
	alSourceQueueBuffers(source, 1, &bufferID);

	audio_pts = frame->audio_clock;//获取音频时间戳

	//释放数据
	if (frame)
	{
		av_free(frame->data);
		delete frame;
	}

	return 0;
}

//画面刷新
int sfp_refresh_thread(int timeInterval, bool& afaster, bool& vfaster)
{
	thread_exit = 0;
	thread_pause = 0;
	while (!thread_exit)
	{
		if (!thread_pause)
		{
			SDL_Event event;
			event.type = REFRESH_EVENT;//刷新视频画面
			SDL_PushEvent(&event);//发送事件
		}

		if (afaster)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval / 2));//2倍速播放
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval));//正常播放
		}
		if (vfaster)
		{
			SDL_Delay(80);//0.5倍速播放
		}
	}
	thread_exit = 0;
	thread_pause = 0;
	//break
	SDL_Event event;
	event.type = BREAK_EVENT;
	SDL_PushEvent(&event);

	return 0;
}

//音量增减
int sfp_control_thread(float& volumn, bool& volumnChange)
{
	const Uint8* state = SDL_GetKeyboardState(NULL);
	bool key_space_down = false;
	bool key_plus_down = false;
	bool key_minus_down = false;
	while (!thread_exit)
	{
		if (state[SDL_SCANCODE_W] && !key_plus_down) {
			key_plus_down = true;
			if (volumn < 50) {
				volumn += 0.1;
			}
		}
		else if (!state[SDL_SCANCODE_W] && key_plus_down) {
			key_plus_down = false;
		}

		if (state[SDL_SCANCODE_S] && !key_minus_down) {
			key_minus_down = true;
			if (volumn > 0) {
				volumn -= 0.1;
			}
		}
		else if (!state[SDL_SCANCODE_S] && key_minus_down) {
			key_minus_down = false;
		}
		volumnChange = (key_plus_down || key_minus_down);

		//更新键盘状态
		state = SDL_GetKeyboardState(NULL);
	}
	return 0;
}

//音频播放同步视频快进
void audio_forward(double second) {
	double new_pts = audio_pts + second;

	while (!audioFrameQueue.empty()) {
		AudioFrame frame = audioFrameQueue.front();
		audioFrameQueue.pop();
		if (frame == nullptr)
			return;
		if (frame->audio_clock >= new_pts) {
			break;
		}
		if (frame) {
			av_free(frame->data);
			delete frame;
		}
	}
}

//视频播放
int playVideo(char* filepath)
{
	AVFormatContext* formatContext;//封装格式上下文结构体
	formatContext = avformat_alloc_context();//初始化
	//打开输入文件
	if (avformat_open_input(&formatContext, filepath, NULL, NULL) != 0)
	{
		cout << "error: couldn't open the input file" << endl;
		return -1;
	}
	//获取文件信息
	if (avformat_find_stream_info(formatContext, NULL) < 0)
	{
		cout << "error: couldn't find the stream information" << endl;
		return -1;
	}
	//查找视频流信息
	int index = -1;
	for (int i = 0; i < formatContext->nb_streams; i++)
	{
		if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
		{
			index = i;
			break;
		}
	}
	if (index == -1)
	{
		cout << "error: couldn't find a video stream" << endl;
		return -1;
	}
	//查找视频解码器
	AVCodecContext* codecContext;
	AVCodec* codec;
	codecContext = avcodec_alloc_context3(NULL);
	avcodec_parameters_to_context(codecContext, formatContext->streams[index]->codecpar);
	codec = avcodec_find_decoder(codecContext->codec_id);
	if (codec == NULL)
	{
		cout << "error: couldn't find video decoder" << endl;
		return -1;
	}
	//打开解码器
	if (avcodec_open2(codecContext, codec, NULL) < 0)
	{
		cout << "error: couldn't open video decoder" << endl;
		return -1;
	}

	AVPacket* packet;//压缩数据
	packet = (AVPacket*)av_malloc(sizeof(AVPacket));
	AVFrame* frame;//解码后数据
	AVFrame* frameYUV;//解码后YUV数据
	frame = av_frame_alloc();
	frameYUV = av_frame_alloc();
	uint8_t* buffer;
	struct SwsContext* convertContext;
	buffer = (uint8_t*)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, codecContext->width, codecContext->height, 1));
	av_image_fill_arrays(frameYUV->data, frameYUV->linesize, buffer, AV_PIX_FMT_YUV420P, codecContext->width, codecContext->height, 1);
	convertContext = sws_getContext(codecContext->width, codecContext->height, codecContext->pix_fmt,
		codecContext->width, codecContext->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);

	//初始化SDL     
	SDL_Window* sdlWindow;//窗口
	SDL_Renderer* sdlRenderer;//渲染器
	SDL_Texture* sdlTexture;//纹理
	SDL_Rect sdlRect;//矩形窗
	SDL_Event event;//事件
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER))
	{
		cout << "error: couldn't initialize SDL" << endl;
		exit(1);
	}
	//创建窗口
	window_w = codecContext->width;
	window_h = codecContext->height;
	sdlWindow = SDL_CreateWindow("Little Player", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, window_w,
		window_h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
	if (!sdlWindow)
	{
		cout << "error: couldn't create SDL window" << endl;
		exit(1);
	}
	//创建渲染器
	sdlRenderer = SDL_CreateRenderer(sdlWindow, -1, 0);
	//创建纹理
	sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING,
		codecContext->width, codecContext->height);

	//设置矩形窗的位置和大小
	sdlRect.x = 0;
	sdlRect.y = 0;
	sdlRect.w = window_w;
	sdlRect.h = window_h;

	double video_pts = 0;
	double diff = 0;
	int ret, picture;

	double frameRate = (double)formatContext->streams[index]->avg_frame_rate.num / formatContext->streams[index]->avg_frame_rate.den;

	bool afaster = false;
	bool vfaster = false;
	std::thread video_tid{ sfp_refresh_thread,(int)frameRate ,std::ref(afaster) ,std::ref(vfaster) };

	while (1)
	{
		while (1)
		{
			if (av_read_frame(formatContext, packet) < 0)
				thread_exit = 1;
			if (packet->stream_index == index)
				break;
		}
		ret = avcodec_send_packet(codecContext, packet);
		picture = avcodec_receive_frame(codecContext, frame);
		if (!picture)
		{
			//去除无效数据
			sws_scale(convertContext, (const uint8_t* const*)frame->data, frame->linesize, 0, codecContext->height, frameYUV->data, frameYUV->linesize);
		}
		while (1)
		{
			SDL_WaitEvent(&event);
			if (event.type == REFRESH_EVENT)
			{
				if (audioFrameQueue.empty())
				{
					sws_freeContext(convertContext);
					SDL_Quit();
					av_frame_free(&frameYUV);
					av_frame_free(&frame);
					avcodec_close(codecContext);
					avformat_close_input(&formatContext);
				}
				if (true)
				{
					video_pts = (double)frame->pts * av_q2d(formatContext->streams[index]->time_base);
					diff = (audio_pts - video_pts) * 1000;
					if (audio_pts > video_pts && diff > 30)
					{
						afaster = true;
						vfaster = false;
					}
					else if (audio_pts < video_pts && diff < -30)
					{
						afaster = false;
						vfaster = true;
					}
					else
					{
						afaster = false;
						vfaster = false;
					}
				}
				//SDL显示
				SDL_UpdateTexture(sdlTexture, NULL, frameYUV->data[0], frameYUV->linesize[0]);
				SDL_RenderClear(sdlRenderer);
				SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);
				SDL_RenderPresent(sdlRenderer);
				av_packet_unref(packet);
				break;
			}
			else if (event.type == SDL_WINDOWEVENT)
			{
				//改变窗口大小
				SDL_GetWindowSize(sdlWindow, &window_w, &window_h);
			}
			else if (event.type == SDL_QUIT)
			{
				thread_exit = 1;
				exit(0);
			}
			else if (event.type == SDL_KEYDOWN)
			{
				switch (event.key.keysym.sym)
				{
				case SDLK_SPACE:
					thread_pause = thread_pause ? false : true;
					break;
				case SDLK_1:
					video_forward = 10.0;
					av_seek_frame(formatContext, index, (video_pts + video_forward) / av_q2d(formatContext->streams[index]->time_base), AVSEEK_FLAG_BACKWARD);
					avcodec_flush_buffers(codecContext);
					forward_state = true;
					break;
				case SDLK_3:
					video_forward = 30.0;
					av_seek_frame(formatContext, index, (video_pts + video_forward) / av_q2d(formatContext->streams[index]->time_base), AVSEEK_FLAG_BACKWARD);
					avcodec_flush_buffers(codecContext);
					forward_state = true;
					break;
				default:
					break;
				}
			}
			else if (event.type == BREAK_EVENT)
				break;
		}
		av_packet_unref(packet);
	}

	//关闭
	sws_freeContext(convertContext);
	SDL_Quit();
	av_frame_free(&frameYUV);
	av_frame_free(&frame);
	avcodec_close(codecContext);
	avformat_close_input(&formatContext);

	return 0;
}



//视音频播放
int playAudioWithVideo(char* filepath)
{
	std::thread videoPlay{ playVideo,filepath };
	videoPlay.detach();

	float volumn = 1.0;
	bool vchange = false;
	bool fast_forward_10 = false;
	bool fast_forward_30 = false;

	AVFormatContext* formatContext;//封装格式上下文结构体
	formatContext = avformat_alloc_context();//初始化
	//打开输入文件
	if (avformat_open_input(&formatContext, filepath, NULL, NULL) != 0)
	{
		cout << "error: couldn't open the input file" << endl;
		return -1;
	}
	//获取文件信息
	if (avformat_find_stream_info(formatContext, NULL) < 0)
	{
		cout << "error: couldn't find the stream information" << endl;
		return -1;
	}
	//以音频播放为依托，视频播放通过另一线程实现
	//查找音频流信息
	int index = -1;
	for (int i = 0; i < formatContext->nb_streams; i++)
	{
		if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
		{
			index = i;
			break;
		}
	}
	if (index == -1)
	{
		cout << "error: couldn't find a audio stream" << endl;
		return -1;
	}
	//查找音频解码器
	AVCodecContext* codecContext;
	AVCodec* codec;
	codecContext = avcodec_alloc_context3(NULL);
	avcodec_parameters_to_context(codecContext, formatContext->streams[index]->codecpar);
	codec = avcodec_find_decoder(codecContext->codec_id);
	if (codec == NULL)
	{
		cout << "error: couldn't find audio decoder" << endl;
		return -1;
	}

	//解决时间戳报错问题
	codecContext->pkt_timebase = formatContext->streams[index]->time_base;

	//打开解码器
	if (avcodec_open2(codecContext, codec, NULL) < 0)
	{
		cout << "error: couldn't open audio decoder" << endl;
		return -1;
	}

	AVPacket* packet;//解码前数据
	packet = (AVPacket*)av_malloc(sizeof(AVPacket));
	AVFrame* frame;//解码数据
	frame = av_frame_alloc();

	SwrContext* swr = swr_alloc();//音频数据重采样
	//----重采样参数设置----start
	enum AVSampleFormat in_sample_fmt = codecContext->sample_fmt;//输入采样格式
	enum AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16;//输出采样格式：16 bits
	int in_sample_rate = codecContext->sample_rate;//输入采样率
	int out_sample_rate = codecContext->sample_rate;//输出采样率：44.1kHz
	uint64_t in_ch_layout = codecContext->channel_layout;//输入声道布局
	uint64_t out_ch_layout = AV_CH_LAYOUT_STEREO;//输出声道布局：双声道
	//----重采样参数设置----end
	swr_alloc_set_opts(swr,
		out_ch_layout, out_sample_fmt, out_sample_rate,
		in_ch_layout, in_sample_fmt, in_sample_rate,
		0, NULL);
	swr_init(swr);

	int nb_out_channel = av_get_channel_layout_nb_channels(out_ch_layout);//输出声道个数

	//PCM数据
	uint8_t* out_buffer;
	int out_buffer_size;
	int ret;
	//循环解码并存储
	while (av_read_frame(formatContext, packet) == 0)
	{
		if (packet->stream_index == index)
		{
			ret = avcodec_send_packet(codecContext, packet);
			if (ret < 0)
			{
				cout << "avcodec_send_packet: " << ret << endl;
				continue;
			}
			while (ret >= 0)
			{
				ret = avcodec_receive_frame(codecContext, frame);
				if (ret == 0)
				{
					out_buffer = (uint8_t*)av_malloc(MAX_AUDIO_FRAME_SIZE);
					//重采样
					swr_convert(swr, &out_buffer, MAX_AUDIO_FRAME_SIZE, (const uint8_t**)frame->data, frame->nb_samples);
					//获取大小
					out_buffer_size = av_samples_get_buffer_size(NULL, nb_out_channel,
						frame->nb_samples, out_sample_fmt, 1);
					AudioFrame aframe = new AUDIOFRAME;//声明新的音频帧
					//赋值
					aframe->data = out_buffer;
					aframe->data_size = out_buffer_size;
					aframe->sample_rate = out_sample_rate;
					aframe->audio_clock = frame->pts * av_q2d(codecContext->time_base);
					//放入队列
					audioFrameQueue.push(aframe);
				}
			}
		}
		av_packet_unref(packet);
	}

	//初始化openAL
	ALCdevice* pDevice = alcOpenDevice(NULL);
	ALCcontext* pContext = alcCreateContext(pDevice, NULL);
	alcMakeContextCurrent(pContext);

	//创建source并配置
	alGenSources(1, &source);
	if (alGetError() != AL_NO_ERROR)
	{
		cout << "error: couldn't generate audio source" << endl;
		return -1;
	}
	ALfloat SourcePos[] = { 0.0,0.0,0.0 };
	ALfloat SourceVel[] = { 0.0,0.0,0.0 };
	ALfloat ListenerPos[] = { 0.0,0.0 };
	ALfloat ListenerVel[] = { 0.0,0.0,0.0 };
	ALfloat ListenerOri[] = { 0.0,0.0,-1.0,0.0,1.0,0.0 };
	alSourcef(source, AL_PITCH, 1.0);
	alSourcef(source, AL_GAIN, 1.0);
	alSourcefv(source, AL_POSITION, SourcePos);
	alSourcefv(source, AL_VELOCITY, SourceVel);
	alSourcef(source, AL_REFERENCE_DISTANCE, 50.0f);
	alSourcei(source, AL_LOOPING, AL_FALSE);

	//创建buffer
	ALuint alBufferArray[NUMBUFFERS];
	alGenBuffers(NUMBUFFERS, alBufferArray);
	ALint iQueuedBuffers;

	std::thread control_openal{ sfp_control_thread,std::ref(volumn),std::ref(vchange) };
	control_openal.detach();

	//首次填充数据
	for (int i = 0; i < NUMBUFFERS; i++)
	{
		feedAudioData(alBufferArray[i]);
	}
	//开始播放
	alSourcePlay(source);

	//若队列不为空则一直播放
	while (!audioFrameQueue.empty())
	{
		if (forward_state)
		{
			audio_forward(video_forward);
			forward_state = false;
			video_forward = 0;
		}

		ALint iBuffersProcessed = 0;
		alGetSourcei(source, AL_BUFFERS_PROCESSED, &iBuffersProcessed);
		while (iBuffersProcessed > 0) {
			ALuint bufferID = 0;
			alSourceUnqueueBuffers(source, 1, &bufferID);
			feedAudioData(bufferID);
			iBuffersProcessed -= 1;
		}
		if (vchange)
		{
			alSourcef(source, AL_GAIN, volumn);
		}
		ALint iState;
		alGetSourcei(source, AL_SOURCE_STATE, &iState);
		if (thread_pause)
		{
			alSourcePause(source);
		}
		else if (iState != AL_PLAYING)
		{
			alGetSourcei(source, AL_BUFFERS_QUEUED, &iQueuedBuffers);
			if (iQueuedBuffers) {
				alSourcePlay(source);
			}
			else {
				break;
			}
		}
		if (thread_exit) {
			alSourceStop(source);
			alSourcei(source, AL_BUFFER, 0);
			alDeleteBuffers(NUMBUFFERS, alBufferArray);
			alDeleteSources(1, &source);
			break;
		}
	}

	//关闭
	alSourceStop(source);
	alSourcei(source, AL_BUFFER, 0);
	alDeleteBuffers(NUMBUFFERS, alBufferArray);
	alDeleteSources(1, &source);

	av_frame_free(&frame);
	swr_free(&swr);

	alcMakeContextCurrent(NULL);
	alcDestroyContext(pContext);
	pContext = NULL;
	alcCloseDevice(pDevice);
	pDevice = NULL;

	return 0;
}


int main(int argc, char* argv[])
{
	cout << "hello, little player" << endl;
	char* filepath = argv[1];
	if (argc != 2)
	{
		cout << "input error: argv[1] should be the media file" << endl;
	}
	else
	{
		cout << "play file: " << filepath << endl;
		avformat_network_init();
		playAudioWithVideo(filepath);
	}
	return 0;
}