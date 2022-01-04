/**
 * @file host camera server.cc
 * @author Shiva Kumara R (shiva.kumara.rudrappa@intel.com)
 * @brief
 * @version 1.0
 * @date 2021-04-30
 *
 * Copyright (c) 2021 Intel Corporation
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
 *
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

typedef struct stream_ctx_t
{
    char *output_path, *output_format;
    AVInputFormat *ifmt;
    AVFormatContext *ifmt_ctx, *ofmt_ctx;
    AVCodec *in_codec, *out_codec;
    AVStream *in_stream, *out_stream;
    AVCodecContext *in_codec_ctx, *out_codec_ctx;
} stream_ctx_t;

stream_ctx_t *stream_ctx;

char device_index[] = "/dev/video0";
int width = 640;
int height = 480;
int fps = 30;
AVPacket *pkt;

#define BUF_COUNT 4
unsigned int buf_count = 0;
unsigned char *buf_list[BUF_COUNT];

void get_all_dev_nodes()
{
    int fd;
    struct v4l2_capability video_cap;
    char dev_name[50];
    for(int devId = 0; devId < 64; devId++) {
        sprintf(dev_name, "/dev/video%d", devId);

        if((fd = open(dev_name, O_RDONLY)) == -1) {
	    continue;
	}

        if(ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == -1)
            cout <<"cam_info: Can't get capabilities\n";
        else {
		cout << "card "<<video_cap.card<<"devId " <<devId<<"\n";
	    if((!strcmp((const char*)video_cap.driver, "v4l2 loopback")) && (!strcmp((const char*)video_cap.card, "normal")))
	        strcpy(device_index, dev_name);
        }
        close(fd);
    }

}

void dumpFrame(unsigned char *bufdest) {
    FILE* pFile;
    char file_name[100] = "output";
    if(buf_count > 30)
	    return;
    unsigned int img_id = buf_count ;
    sprintf(file_name, "%d.yuv", img_id);
    pFile = fopen(file_name,"wb");

    if (pFile ){
        fwrite(bufdest,1,width * height * 1.5,pFile);
    }
    else
        cout << "Can't open file\n";

    if(pFile)
        fclose(pFile);

}

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
    av_dict_set(&options, "pixel_format", "yuv420p", 0);
    av_dict_set(&options, "probesize", "7000000", 0);

    if (avformat_open_input(&stream_ctx->ifmt_ctx, device_index, NULL, NULL/*stream_ctx->ifmt, &options*/) != 0)
    {
        cout<<"cannot initialize input device! "<<av_strerror<<"\n";;
        ret_code = 1;
    }

    avformat_find_stream_info(stream_ctx->ifmt_ctx, 0);
    return 0;
}



int open_camera()
{
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

int main(int argc, char** argv)
{
    atomic<bool> stop = true;
    int          instance_id = 3;
    thread       file_src_thread;
    atomic<bool> request_negotiation = false;
    static int open_close_count = 0;
	//search for virtual device nodes
    char sys_path[255];

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


    unsigned int temp = 0;
    cout <<"open camera " << device_index;


    pkt = av_packet_alloc();
    av_init_packet(pkt);
    buf_count = 0;
    //create buffer
    const size_t inbuf_size = width * height * 1.5;
    
    shared_ptr<VideoSink>   video_sink;

    VsockConnectionInfo conn_info = { instance_id };
    try {
        video_sink = make_shared<VideoSink>(conn_info);
    } catch (const std::exception& ex) {
        cout << "VideoSink creation error :"
             << ex.what() << endl;
        exit(1);
    }

    cout << "[Stream] Waiting Camera Open callback..\n" << device_index;

    video_sink->RegisterCallback(
      [&](const VideoSink::CtrlMessage& ctrl_msg) {
          cout << "[Stream] received new cmd to process ";

          switch (ctrl_msg.cmd) {
              case VideoSink::Command::kOpen:
	          cout << "[Stream] Received Open command from Camera VHal\n";
		  //cout <<"[Stream] camera open called again "<<width<<height<<"\n";
		  buf_count = 0;
		  if( open_close_count % 2 != 0) {
		  cout << "[Stream] camera already opened, closing old instance"<<"\n";

                  stop = true;
                  this_thread::sleep_for(100ms);
                  for(int count = 0; count < BUF_COUNT; count++)
                      free(buf_list[count]);

		  avformat_close_input(&stream_ctx->ifmt_ctx);
                  avformat_close_input(&stream_ctx->ofmt_ctx);
                  free(stream_ctx);
                  stream_ctx = NULL;
                  file_src_thread.join();
                  open_close_count++;

		  }
                  stop = false;
                  for(int count = 0; count < BUF_COUNT; count++)
                      buf_list[count] = (unsigned char*)calloc(1, inbuf_size);
                  open_camera();
                  open_close_count++;
                  file_src_thread = thread([&stop,
                                            &video_sink,
                                            &device_index]() {

                     const size_t inbuf_size = width * height * 1.5;
                      while (!stop) {
		      
                          if(av_read_frame(stream_ctx->ifmt_ctx, pkt) < 0)
                              cout << "[Stream] Fail to read frame";
                          yuyv422_to_yuv420sp(pkt->data, buf_list[buf_count % BUF_COUNT], width, height, false);
                          // Write payload
                          if (auto [sent, error_msg] =
                                video_sink->SendRawPacket(buf_list[buf_count % BUF_COUNT],
                                                            inbuf_size);
                              sent < 0) {
                              cout <<"[Stream] closing camera as packet send failed: "
                                << error_msg << "\n";
                          }
                          buf_count++;
                          this_thread::sleep_for(33ms);
			  av_packet_unref(pkt);
                          av_new_packet(pkt, 0);

                      }

                  });
                  break;

              case VideoSink::Command::kClose:
                  if( open_close_count % 2 == 0) {
			  cout <<"[Stream] camera already closed "<<endl;
                  }
                  stop = true;
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

             case VideoSink::Command::kNone:
                  cout << "Received None\n";
                  break;

              default:
                  cout << "Unknown Command received, exiting with failure : "  << (int)ctrl_msg.cmd << "\n";
                  break;
          }
      });

    // we need to be alive :)
    while (true) {
        this_thread::sleep_for(33ms);
    }

    return 0;
}
