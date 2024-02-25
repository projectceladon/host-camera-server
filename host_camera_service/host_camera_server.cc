/*
 *@file: Host Camera Server Code to support multiple cameras
 *@author: Venkatesh Pillai (venkatesh.pillai@intel.com)
 *@date: 17 December 2022
 *@version 2.0
 *
 *
 * Copyright (c) 2022 Intel Corporation
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "vsock_stream_socket_client.h"
#include "video_sink.h"
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libavdevice/avdevice.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

using namespace std::chrono_literals;
using namespace vhal::client;
using namespace std;


#define NUM_OF_CAMERAS_REQUESTED 2


//Defining the structures

typedef struct stream_ctx_t
{
    char *output_path, *output_format;
    AVInputFormat *ifmt;
    AVFormatContext *ifmt_ctx, *ofmt_ctx;
    AVCodec *in_codec, *out_codec;
    AVStream *in_stream, *out_stream;
    AVCodecContext *in_codec_ctx, *out_codec_ctx;
} stream_ctx_t;

struct pixel_yvuv {
    uint8_t Y0;
    uint8_t V;
    uint8_t Y1;
    uint8_t U;
};

struct pixel_NV21_y_plane{
    uint8_t Y0;
    uint8_t Y1;
    uint8_t Y2;
    uint8_t Y3;
};

struct pixel_NV21_uv_plane{
    uint8_t data;
};



//Declaring the varaibles

thread       file_src_thread;
struct timeval topen_recv, tclose_recv;

shared_ptr<VideoSink>   video_sink;

pthread_cond_t thread_running = PTHREAD_COND_INITIALIZER;
pthread_mutex_t thread_lock = PTHREAD_MUTEX_INITIALIZER;

stream_ctx_t *stream_ctx;

int numOfCameras;

atomic<bool> stop = true;

static int open_close_count = 0;

char* device_index[] = {"/dev/video0","/dev/video2"};
//char device_index1[] = "/dev/video0";

int width = 1920;
int height = 1080;
int fps = 30;
AVPacket *pkt;
int v4l2_format = VideoSink::VideoCodecType::kMJPEG;
#define BUF_COUNT 4
unsigned int buf_count = 0;
unsigned char *buf_list[BUF_COUNT];

pthread_mutex_t mMainLock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t mSignalMain = PTHREAD_COND_INITIALIZER;


//Function Implementation

void
yuyv422_to_yuv420sp(unsigned char *bufsrc, unsigned char *dst_buf, int width, int height, bool flipuv)
{
    int i,j;

    volatile struct pixel_yvuv* src;
    volatile struct pixel_NV21_y_plane* dst_p1;
    volatile struct pixel_NV21_uv_plane* dst_p2;
    volatile struct pixel_NV21_uv_plane* dst_p3;
    src = (struct pixel_yvuv*)bufsrc;
    /* plane 1 */
    dst_p1 = (struct pixel_NV21_y_plane*) dst_buf;
    /* plane 2 */
    dst_p2 = (struct pixel_NV21_uv_plane*) (dst_buf + (height*width)); /* offset to UV plane */
    unsigned int v_plane = (height*width) + (height * width * 0.25);
    dst_p3 = (struct pixel_NV21_uv_plane*) (dst_buf + v_plane); /* offset to UV plane */

    for(i=1; i<=height; i++) {
        for(j=1; j<=width/2; j++) {
            if(j%2) {
                dst_p1->Y0 = src->Y0;
                dst_p1->Y1 = src->Y1;
            } else {
                dst_p1->Y2 = src->Y0;
                dst_p1->Y3 = src->Y1;
                 dst_p1++;
            }

            /* vertical subsampling for U and V plane */
            if(i%2) {
             /* U and V  Plane */
                if(flipuv) {
                    dst_p2->data  = src->U;
                    dst_p3->data  = src->V;
                } else {
                    dst_p2->data  = src->V;
                    dst_p3->data  = src->U;
                }
                dst_p2++;
                dst_p3++;
            }
            src++;
        }
    }
}


const char *get_device_family()
{
#ifdef _WIN32
  const char *device_family = "dshow";
#elif __APPLE__
  const char *device_family = "avfoundation";
#elif __linux__
  const char *device_family;
          device_family = "v4l2";
#endif

  return device_family;
}

int init_device_and_input_context(stream_ctx_t *stream_ctx, const char *device_family, const char *device_index, int width, int height, int fps)
{

    int ret_code = 0;

    std::string fps_str = std::to_string(fps);
    std::string size = std::to_string(width) + std::string("x") + std::to_string(height);

    stream_ctx->ifmt = (AVInputFormat *)av_find_input_format(device_family);
    AVDictionary *options = NULL;
    av_dict_set(&options, "video_size", size.c_str(), 0);
    av_dict_set(&options, "framerate", fps_str.c_str(), 0);
    av_dict_set(&options, "probesize", "7000000", 0);
//    if(v4l2_format == VideoSink::VideoCodecType::kI420)
  //      av_dict_set(&options, "input_format", "yuv", 0); 
    if(v4l2_format == VideoSink::VideoCodecType::kMJPEG)
        av_dict_set(&options, "input_format", "mjpeg", 0);
    if(v4l2_format == VideoSink::VideoCodecType::kH264)
        av_dict_set(&options, "input_format", "h264", 0);

    if (avformat_open_input(&stream_ctx->ifmt_ctx, device_index, NULL, &options) != 0)
    {
        cout<<"cannot initialize input device! "<<av_strerror<<"\n";;
        ret_code = 1;
    }

    avformat_find_stream_info(stream_ctx->ifmt_ctx, 0);
    return 0;
}
/*
inline int doStream()
{

                      while (!stop) {
                          gettimeofday(&tval_before, NULL);

                          if(av_read_frame(stream_ctx->ifmt_ctx, pkt) < 0)
                              cout << "[Stream] Fail to read frame";
                          //dumpFrame(pkt->data, pkt->size);
                          if(v4l2_format == VideoSink::VideoCodecType::kI420) {
                              yuyv422_to_yuv420sp(pkt->data, buf_list[buf_count % BUF_COUNT], width, height, false);

                              // Write payload
                              if (auto [sent, error_msg] =
                                  video_sink->SendDataPacket(buf_list[buf_count % BUF_COUNT],
                                     inbuf_size);
                                  sent < 0) {
                                        cout <<"[Stream] closing camera as packet send failed: "
                                            << error_msg << "\n";
                              }

                          }else {
                              if (auto [sent, error_msg] =
                                video_sink->SendDataPacket(pkt->data,
                                                           pkt->size);
                                 sent < 0) {
                                    cout <<"[Stream] closing camera as packet send failed: "
                                        << error_msg << "\n";
                                 }
                          }
                          buf_count++;
                          gettimeofday(&tval_after, NULL);
                          timersub(&tval_after, &tval_before, &tval_result);
                          unsigned int sleep_time = (33 - (tval_result.tv_usec /1000));
                          std::chrono::duration<double, std::milli> elapsed = (std::chrono::duration<double, std::milli>)sleep_time;


                          //this_thread::sleep_for(33ms);
                          av_packet_unref(pkt);
                          av_new_packet(pkt, 0);

                          //if(sleep_time > 1)
                          this_thread::sleep_for(32ms);
                      }

   return stop?0:1;
}
*/
 int open_camera(char* device_index)
{
	cout<<"Camera is being opened "<<device_index<<endl;
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    av_register_all();
#endif
    avdevice_register_all();

    const char *device_family = get_device_family();

    stream_ctx = (stream_ctx_t *)malloc(sizeof(stream_ctx_t));

    if(!stream_ctx)
        return -1;

    stream_ctx->ifmt = NULL;
    stream_ctx->ifmt_ctx = NULL;
    stream_ctx->ofmt_ctx = NULL;
    stream_ctx->out_codec = NULL;
    stream_ctx->out_stream = NULL;
    stream_ctx->out_codec_ctx = NULL;

        if (init_device_and_input_context(stream_ctx, device_family, device_index, width, height, fps) != 0)
    {
        return -1;
    }
    return 0;
 }


int close_camera()
{
                  if( open_close_count % 2 == 0) {
                          cout <<"[Stream] camera already closed "<<endl;
                  }
                  stop = true;
                  pthread_cond_wait(&thread_running, &thread_lock);
                  pthread_mutex_unlock(&thread_lock);
                  this_thread::sleep_for(100ms);
                  for(int count = 0; count < BUF_COUNT; count++)
                      free(buf_list[count]);

                  cout << "[Stream] Received Close command from Camera VHal\n";
                  avformat_close_input(&stream_ctx->ifmt_ctx);
                  avformat_close_input(&stream_ctx->ofmt_ctx);
                  free(stream_ctx);
                  stream_ctx = NULL;

                  file_src_thread.join();
                  open_close_count++;
              
    return 0;
}

void* InitCamera(void *arg)
{
    while(true) {
        video_sink->ResetCameraCapabilty();
        cout <<"[Stream] start capabilty exchange";
        video_sink->GetCameraCapabilty();
        std::vector<VideoSink::camera_info_t> camera_info(NUM_OF_CAMERAS_REQUESTED);
        for (int i = 0; i < NUM_OF_CAMERAS_REQUESTED; i++) {
            camera_info[i].codec_type = (VideoSink::VideoCodecType)v4l2_format;
            camera_info[i].resolution = VideoSink::FrameResolution::k1080p;
        }
        video_sink->SetCameraCapabilty(camera_info);
    }
}
#if 1
int main(int argc, char** argv)
{

	static int track=0;
	int instance_id = 3;
//thread       file_src_thread;
//atomic<bool> request_negotiation = false;
//search for virtual device nodes
	char sys_path[255];
	cout <<"Venkatesh host_camera_server in main"<<endl;
	pthread_t t_handle;
#if 0
    for(int devId = 0; devId < 255; devId++) {
        sprintf(sys_path,"/sys/devices/virtual/video4linux/video%d/name", devId);
        int fp = open(sys_path, O_RDONLY);
        if(fp) {
            char sys_entry[12];
            read(fp, sys_entry, 10);
            if(strstr(sys_entry, "normal") != NULL) {
                cout <<"found virtual node"<<sys_entry<<endl;
                char dev_name[255];
                sprintf(dev_name, "/dev/video%d", devId);
                strcpy(device_index, dev_name);
                break;
            }
        }
    }
#endif

	unsigned int temp = 0;
    //cout <<"open camera " << device_index<<endl;

	pkt = av_packet_alloc();
	av_init_packet(pkt);
	buf_count = 0;
    //create buffer
	const size_t inbuf_size = width * height * 1.5;

    VsockConnectionInfo conn_info = { instance_id };
    try {
        video_sink = make_shared<VideoSink>(conn_info,
          [&](const VideoSink::camera_config_cmd_t& ctrl_msg) {
            cout << "[Stream] received new cmd to process ";

          switch (ctrl_msg.cmd) {
                case VideoSink::camera_cmd_t::CMD_OPEN:
                  cout << " [Stream] Received Open command from Camera VHal\n";
                  buf_count = 0;
                  if( open_close_count % 2 != 0) {
                  cout << "[Stream] camera already opened, closing old instance"<<"\n";
this_thread::sleep_for(32ms);
/*if(!stop){
                  stop = true;
                  pthread_cond_wait(&thread_running, &thread_lock);
                  pthread_mutex_unlock(&thread_lock);

                  this_thread::sleep_for(100ms);
                  for(int count = 0; count < BUF_COUNT; count++)
                      free(buf_list[count]);

                  avformat_close_input(&stream_ctx->ifmt_ctx);
                  avformat_close_input(&stream_ctx->ofmt_ctx);
                  free(stream_ctx);
                  stream_ctx = NULL;
                  file_src_thread.join();

		 // open_camera(device_index[1]);
		 // cout<<"Opening New Camera "<<device_index[1]<<endl;
        
}*/
                  }


if(!stop){
                  stop = true;
                  pthread_cond_wait(&thread_running, &thread_lock);
                  pthread_mutex_unlock(&thread_lock);

                  this_thread::sleep_for(100ms);
                  for(int count = 0; count < BUF_COUNT; count++)
                      free(buf_list[count]);

                  avformat_close_input(&stream_ctx->ifmt_ctx);
                  avformat_close_input(&stream_ctx->ofmt_ctx);
                  free(stream_ctx);
                  stream_ctx = NULL;
                  file_src_thread.join();

                 // open_camera(device_index[1]);
                 // cout<<"Opening New Camera "<<device_index[1]<<endl;

}

                  pthread_mutex_lock(&thread_lock);
                  stop = false;
                  for(int count = 0; count < BUF_COUNT; count++)
                      buf_list[count] = (unsigned char*)calloc(1, inbuf_size);

    		  open_camera(device_index[open_close_count%NUM_OF_CAMERAS_REQUESTED]);
                  open_close_count++;
                  file_src_thread = thread([&stop,
                                            &video_sink,
                                            &device_index]() {

                     const size_t inbuf_size = width * height * 1.5;

                    struct timeval tval_before, tval_after, tval_result;
/*                   if(doStream())
		      cout<<"Streaming encountered an error "<<endl;
		   else
		      cout<<"Stream was closed by camera Vhal"<<endl;
*/
		    gettimeofday(&topen_recv, NULL);
                      while (!stop) {
                          gettimeofday(&tval_before, NULL);

                          if(av_read_frame(stream_ctx->ifmt_ctx, pkt) < 0)
                              cout << "[Stream] Fail to read frame";
                          //dumpFrame(pkt->data, pkt->size);
                          if(v4l2_format == VideoSink::VideoCodecType::kI420) {
                              yuyv422_to_yuv420sp(pkt->data, buf_list[buf_count % BUF_COUNT], width, height, false);

                              // Write payload
                              if (auto [sent, error_msg] =
                                  video_sink->SendDataPacket(buf_list[buf_count % BUF_COUNT],
                                     inbuf_size);
                                  sent < 0) {
                                        cout <<"[Stream] closing camera as packet send failed: "
                                            << error_msg << "\n";
                              }

                          }else {
                              if (auto [sent, error_msg] =
                                video_sink->SendDataPacket(pkt->data,
                                                           pkt->size);
                                 sent < 0) {
                                    cout <<"[Stream] closing camera as packet send failed: "
                                        << error_msg << "\n";
                                 }
                          }
                          buf_count++;
                          gettimeofday(&tval_after, NULL);
timersub(&tval_after, &tval_before, &tval_result);
unsigned int sleep_time = (33 - (tval_result.tv_usec /1000));
    std::chrono::duration<double, std::milli> elapsed = (std::chrono::duration<double, std::milli>)sleep_time;


//this_thread::sleep_for(33ms);
                          av_packet_unref(pkt);
                          av_new_packet(pkt, 0);

//if(sleep_time > 1)
this_thread::sleep_for(32ms);
          }


	cout <<"camera thread exit "<<"\n";
                      pthread_cond_signal(&thread_running);
                  });
                  break;

                case VideoSink::camera_cmd_t::CMD_CLOSE:

	      struct timeval tval_res;	
		gettimeofday(&tclose_recv, NULL);
		timersub(&topen_recv, &tclose_recv, &tval_res);
    cout<<"Time Diff between open and close "<<(unsigned int)tval_res.tv_usec <<endl;

    if(tval_res.tv_usec<1000000)
{
	cout<<"Skipping Close command"<<endl;
	break;
}
/*
if(close_camera)
	cout<<"Camera Close failed "<<endl;
else
	cout<<"Camera Close Done "<<endl;
*/
 		  if( open_close_count % 2 == 0) {
                          cout <<"[Stream] camera already closed "<<endl;
                  }
                  stop = true;
                  pthread_cond_wait(&thread_running, &thread_lock);
                  pthread_mutex_unlock(&thread_lock);
                  this_thread::sleep_for(100ms);
                  for(int count = 0; count < BUF_COUNT; count++)
                      free(buf_list[count]);

                  cout << "[Stream] Received Close command from Camera VHal\n";
                  avformat_close_input(&stream_ctx->ifmt_ctx);
                  avformat_close_input(&stream_ctx->ofmt_ctx);
                  free(stream_ctx);
                  stream_ctx = NULL;

                  file_src_thread.join();
                  open_close_count++;
                  break;

               case VideoSink::camera_cmd_t::CMD_NONE:
                    cout << "Received None\n";
                    break;

              default:
                  cout << "Unknown Command received, exiting with failure : "  << (int)ctrl_msg.cmd << "\n";
                  break;
          }
      });

    } catch (const std::exception& ex) {

        cout << "VideoSink creation error :"
             << ex.what() << endl;
        exit(1);
    }
    int err = pthread_create(&t_handle, NULL, &InitCamera, NULL);
    if(err != 0)
        cout << "Fail to create thread\n";

    // we need to be alive :)
    while (true) {
        pthread_mutex_lock(&mMainLock);
        pthread_cond_wait(&mSignalMain, &mMainLock);
    }

    return 0;
}
#endif                                                                                                                                                                                                                                                                                                                                                           502,1         Bot
