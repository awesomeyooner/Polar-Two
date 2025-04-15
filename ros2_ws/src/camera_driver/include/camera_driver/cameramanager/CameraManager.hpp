#ifndef CAMERAMANAGER_HPP
#define CAMERAMANAGER_HPP

#include <iostream>
#include <ctime>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"

class CameraManager{

    public:

        CameraManager(){}

        CameraManager(int index) : cap(index){
            this->index = index;
            this->name = "camera_" + index;
            // config();
        }

        CameraManager(int index, cv::VideoCaptureAPIs type) : cap(index, type) {
            this->index = index;
            this->name = "camera_" + index;
            // config();
        }

        CameraManager(std::string path) : cap(path){
            this->path = path;
            this->name = "camera_" + path;
        }
        
        CameraManager(std::string path, cv::VideoCaptureAPIs type) : cap(path, type){
            this->path = path;
            this->name = "camera_" + path;
        }
        
        void config(){   
            config(1280, 960, 50);
        }

        void config(int width, int height, int fps){
            setProperty(cv::CAP_PROP_FRAME_WIDTH, width, true);
            setProperty(cv::CAP_PROP_FRAME_HEIGHT, height, true);
            setProperty(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'), true);
            setProperty(cv::CAP_PROP_FPS, fps, true);
        }

        void config(int width, int height){
            config(width, height, 50);
        }

        bool setProperty(cv::VideoCaptureProperties property, double value, bool print){
            cap.set(property, value);
            return getProperty(property, print) == value;
        }

        bool setPropertyAck(cv::VideoCaptureProperties property, double value, int repeat){

            for(int i = 0; i < repeat; i++){
                bool status = setProperty(property, value, false);

                if(status)
                    return true;
            }

            return false;
        }

        double getProperty(cv::VideoCaptureProperties property, bool print){
            double value = cap.get(property);
            
            if(print)
                std::cout << value << std::endl;

            return value;
        }

        bool isOpened(){
            return cap.isOpened();
        }

        cv::Mat getFrame(){

            previousTimestamp = std::chrono::high_resolution_clock::now();

            cv::Mat frame;
            cap >> frame;

            return frame;
        }

        cv::Mat getFrameGray(){
            cv::Mat frame = getFrame();

            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            return frame;
        }

        double getFramerate(){
            return framerate;
        }

        void updateFramerate(){
            auto now = std::chrono::high_resolution_clock::now();
            auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - previousTimestamp).count();

            framerate = 1 / (delta / 1000000.0);
        }

        bool show(){
            return show(getFrame());
        }

        bool show(cv::Mat frame){

            if(frame.empty())
                return false;

            cv::imshow("frame", frame);

            return true;
        }

        void release(){
            cap.release();
        }

        void reopen(){
            cap.open(index);
        }

    private:
        cv::VideoCapture cap;
        int index;
        std::string path;
        std::string name;

        double framerate = 0;
        std::chrono::_V2::system_clock::time_point previousTimestamp = std::chrono::high_resolution_clock::now();
};

#endif