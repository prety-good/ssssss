//#define get_best_num
//#define save_param
#define calculate

#ifdef get_best_num
#include <QApplication>
#include "tdtcommon.h"
#include "number_detector.h"


#ifdef calculate
#include <algorithm>
#include <iomanip>
#include <sys/time.h>
#include <sys/resource.h>
#endif //calculate

#ifdef save_param
#include <tdtcamera.h>
#include <usart.h>
#include "Debug.h"
#include "armor_detector.h"
#include "armor_resolver.h"
#include "robot_decision.h"
#include "log.h"
#endif //save_param

void ProgOnImage(){
#ifdef save_param
    tdtcamera::VideoDebug vdCamera("../../config/camera_param.yaml");
    tdtcamera::TImage frame;

    tdttoolkit::ReceiveMessage receiveMessage;

    tdtrobot::ArmorDetector armorDetector;
    tdtrobot::ArmorResolver armorResolver;

    tdtdecision::RobotDecision robotDecision;

    tdtusart::RealUsart realUsart;
    tdtusart::Usart& usart = realUsart;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

    vdCamera.GetImage(frame);
    float width=frame.cvimage_.cols;
    float height=frame.cvimage_.rows;
    struct param{
        float comment=0;
    };

    std::vector<std::vector<float>> out_vec;

    cv::FileStorage fs("../../tdt_decision/config.yaml", cv::FileStorage::WRITE);
    if (!fs.isOpened())TDT_FATAL("Can not open the file.");

    int times=1;
    while(times)
    {
        vdCamera.GetImage(frame);
        if(frame.cvimage_.empty())
        {
            std::cout << "frame is empty!" << std::endl;
            #ifdef save_param
            std::cout<<"Out put map!"<<std::endl;
            fs << "map" << out_vec ;
            fs.release();
            #endif //save_param
            continue;
        }
        usart.UsartRecv(receiveMessage);

        auto armors = armorDetector.Get(frame.cvimage_, receiveMessage);
//        std::cout<<"size:"<<armors.size()<<std::endl;
        if(armors.size()>1){
            param params[armors.size()];
            /*****************排序*********************/
            for(unsigned long int i=0;i<armors.size()-1;i++){
                for(unsigned long int j=0;j<armors.size();j++){
                    if(armors[i].GetStickerRect().GetCenter2f().x
                       >armors[j].GetStickerRect().GetCenter2f().x){
                        tdttoolkit::RobotArmor temp=armors[i];
                        armors[i]=armors[j];
                        armors[j]=temp;
                    }
                }
            }

            /*****************画图*********************/
            for(unsigned long int i=0;i<armors.size();i++) {
                cv::putText(frame.cvimage_, std::to_string(i),
                            armors[i].GetStickerRect().GetCenter2f() -
                            cv::Point2f(armors[i].GetStickerRect().GetWidth()/3, armors[i].GetStickerRect().GetHeight()/3),
                            0, 1, cv::Scalar(0, 255, 0));
                cv::circle(frame.cvimage_, armors[i].GetStickerRect().GetCenter2f(), 3, cv::Scalar(255, 255, 255), 3);
            }
            cv::imshow("test",frame.cvimage_);

            /*****************检测键盘输入，决定最优装甲板或输出这一帧之前的数据******************/
            loop:
            int temp=cv::waitKey(0)-176;
            if(temp<armors.size()){
                params[temp].comment=1;
            }
            else{
//                cv::FileStorage fs("../../tdt_decision/armor_decide.yaml", cv::FileStorage::WRITE);//TODO::模块化
                fs << "map" << out_vec ;
                fs.release();
                std::cout<<"Out put map! Please choose an armor."<<std::endl;
                goto loop;
            }

            /*****************保存这一帧的装甲板信息*********************/
            std::vector<float> temp_vec;//存储当前帧的装甲板,每帧重新初始化并赋值
            std::cout<<"-------------------------------"<<times<<"--------------------------------"<<std::endl;
            for(unsigned long int i=0;i<armors.size();i++){
                temp_vec.push_back(armors[i].GetStickerRect().GetAngle());
                temp_vec.push_back(armors[i].GetStickerRect().GetWidth()*
                            armors[i].GetStickerRect().GetHeight());
                temp_vec.push_back(armors[i].GetStickerRect().GetHeight()/
                             armors[i].GetStickerRect().GetWidth());
                temp_vec.push_back(fabs(armors[i].GetStickerRect().GetCenter2f().x - width/2));
                temp_vec.push_back(fabs(armors[i].GetStickerRect().GetCenter2f().y - height/2));
                temp_vec.push_back(params[i].comment);

                std::cout<<"armor "<<i<<":"
                <<" angle:"<<temp_vec[6*i]
                <<" area:"<<temp_vec[6*i+1]
                <<" radio:"<<temp_vec[6*i+2]
                <<" x_diff:"<<temp_vec[6*i+3]
                <<" y_diff:"<<temp_vec[6*i+4]
                <<"comment:"<<temp_vec[6*i+5]<<std::endl;
            }
            out_vec.push_back(temp_vec);
            std::cout<<"----------------------------------------------------------------"<<std::endl;
            times++;
        }
        else{
            cv::imshow("test",frame.cvimage_);
            cv::waitKey(1);
        }

    }
    #pragma clang diagnostic pop
#endif //save_param

#ifdef calculate
//    std::cout<<"The priority is:"<<getpriority(PRIO_PROCESS,getpid())<<std::endl;
//    nice(-20);
//    std::cout<<"The new priority is:"<<nice(-20);

    YAML::Node node = YAML::LoadFile("../../tdt_decision/armor_decide.yaml");//一种读取文件的方式
    const std::vector<std::vector<float>>in_vec = node["map"].as<std::vector<std::vector<float>>>();

    const float sum=in_vec.size();//一共的帧数
    float accuracy=0;
    float coefficient[5]{};
    int calculated=0;
    float result=0.95;

    float correct=0;

    const float an=3.42,ar=0.06,ra=107,xd=0.06,yd=0.3;//angle:3.42 | area:0.06 | radio:107.00 | x_diff:0.06 | y_diff:0.30 | accuracy:0.95440
    const float aa=0.4,bb=0.02,cc=10,dd=0.02,ee=0.1;//范围
    const float time_a=10,time_b=10,time_c=20,time_d=10,time_e=100;////0代表不变
    const double total=(2*time_a+1)*(2*time_b+1)*(2*time_c+1)*(2*time_d+1)*(2*time_e+1);////一共的循环次数（1、3、5、7、9……）
    const float t_1= 2*time_a+1;
    const float t_2= 2*time_b+1;
    const float t_3= 2*time_c+1;
    const float t_4= 2*time_d+1;
    const float t_5= 2*time_e+1;

    std::cout<<std::setiosflags(std::ios_base::fixed);

    float temp_a= t_1==1 ? an : an-aa;
    for(int a=0;a<t_1;a++){
        float temp_b=t_2==1 ? ar : ar-bb;
        for(int b=0;b<t_2;b++){
            float temp_c= t_3==1 ? ra : ra-cc;
            for(int c=0;c<t_3;c++){
                float temp_d= t_4==1 ? xd : xd-dd;
                for(int d=0;d<t_4;d++){
                    float temp_e=t_5==1 ? yd : yd-ee;
                    for(int e=0;e<t_5;e++){
                        calculated++;

                        correct=0;//TODO:在类中 申请为成员变量

                        for(auto & vec : in_vec){
                            float marks[vec.size()/6];std::memset(marks,0,sizeof(marks));
                            int results[vec.size()/6];std::memset(results,0,sizeof(results));

//                            std::cout<<"----------------------------------------------------------------"<<std::endl;
                            for(unsigned long int j=0;j<vec.size();j+=6){
                                marks[j/6]=-temp_a*vec[j]+temp_b*vec[j+1]+temp_c*vec[j+2]-temp_d*vec[j+3]-temp_e*vec[j+4];
                                results[j/6]=vec[j+5];

//                                std::cout<<"armor "<<j<<":"
//                                         <<" angle:"<<vec[j]
//                                         <<" area:"<<vec[j+1]
//                                         <<" radio:"<<vec[j+2]
//                                         <<" x_diff:"<<vec[j+3]
//                                         <<" y_diff:"<<vec[j+4]
//                                         <<" comment:"<<vec[j+5]
//                                         <<" mark:"<<marks[j/6]<<std::endl;
                            }

                            if(std::max_element(marks,marks+vec.size()/6)-marks==std::max_element(results,results+vec.size()/6)-results) {
                                correct++;   ////判断一帧内结果是否正确
                            }

//                            for(int k=0;k<vec.size()/6;k++){
//                                std::cout<<"The "<<k<<"th marks:"<<marks[k]<<", result:"<<results[k]<<std::endl;////输出一帧内各个分数
//                            }
//                            std::cout<<std::max_element(marks,marks+vec.size()/6)-marks<<" "<<std::max_element(results,results+vec.size()/6)-results<<std::endl;////输出一帧内计算与设定的最优结果位置

                        }

                        if(accuracy<correct/sum){////发现更大正确率时保存

                            accuracy=correct/sum;
                            if(accuracy>0.99)goto loop;

                            coefficient[0]=temp_a;
                            coefficient[1]=temp_b;
                            coefficient[2]=temp_c;
                            coefficient[3]=temp_d;
                            coefficient[4]=temp_e;
                        }

                        std::cout<<std::setprecision(4)<<"----------------------------------------------------------"<<std::endl
                                                         <<"angle:"<<temp_a
                                                         <<" area:"<<temp_b
                                                         <<" radio:"<<temp_c
                                                         <<" x_diff:"<<temp_d
                                                         <<" y_diff:"<<temp_e<<std::endl;////这一次循环的参数

                        std::cout<<std::setprecision(0)<<calculated<<"/"<<total<<"     "<<"Corrected radio:"<<correct<<"/"<<sum
                                 <<std::setprecision(15)<<" Accuracy:"<<correct/sum<<std::endl;////这一次循环的正确数/总帧数  已执行次数/总数  正确率

                        if(time_e)temp_e+=ee/time_e;
                    }
                    if(time_d)temp_d+=dd/time_d;
                }
                if(time_c)temp_c+=cc/time_c;
            }
            if(time_b)temp_b+=bb/time_b;
        }
        if(time_a)temp_a+=aa/time_a;
    }
loop:
    std::cout<<std::endl<<"********************************************************************************************************************"<<std::endl
             <<"*Best coefficient:"<<std::setprecision(2)
             <<" | angle:"<<coefficient[0]
             <<" | area:"<<coefficient[1]
             <<" | radio:"<<coefficient[2]
             <<" | x_diff:"<<coefficient[3]
             <<" | y_diff:"<<coefficient[4]
             <<std::setprecision(5)<<" | accuracy:"<<accuracy
             <<std::endl<<"********************************************************************************************************************"<<std::endl;

#endif //calculate
}
int main(int argc, char** argv){
#ifdef save_param
    LoadParam::Init("../../config/robot_param.yaml");
    tdtml::NumberDetector::Init(new tdtml::MxnetNumberDetector());
    tdttoolkit::Time::Init();
    tdtlog::Log::Init();
    tdtlog::Log::AddChannelList("ArmorDetector");
#endif //save_param

    ProgOnImage();
    return 0;
}
#endif //get_best_num
