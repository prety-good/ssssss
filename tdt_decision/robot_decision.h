#ifndef TDTVISION_RM2020_ROBOT_DECISION_H
#define TDTVISION_RM2020_ROBOT_DECISION_H
/*
 * @Name: Decision
 * @Description: 击打决策
 * @Version: 1.0.0.0
 * @Author: 杨泽旭
 * @Date: 2020-2-14
 * @LastEditors:
 * @LastEditTime:
 */
#include <iostream>
#include <opencv2/opencv.hpp>

#include "tdtcommon.h"
namespace tdtdecision {
    /**
    * @name: Decision
    * @description: 用于击打决策
    * @Author: 杨泽旭
    */
    class RobotDecision{
    public:
        RobotDecision();
        ~RobotDecision(){};
        /**
        * @name: Decision::GetBestArmor
        * @description: 进行决策判断
        * @param: 由ArmorResolver得到的 std::vector<tdttoolkit::ResolvedArmor>
        * @return: 一个tdttoolkit::ResolvedArmor
        */
        tdttoolkit::ResolvedArmor ArmorDecision(std::vector<tdttoolkit::ResolvedArmor> &armors);

        /**
       * @name: ClassifyArmors
       * @description: 进行装甲板分类
       * @param: 输入装甲板，输出分类好的装甲板
       * @return: 一个tdttoolkit::ResolvedArmor
       */
        std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor>> ClassifyArmors(std::vector<tdttoolkit::RobotArmor> &input_armors);

        /**
         * @name: RobotDecide
         * @description: 自动选择击打的机器人类型
         * @param: 输入分类且后的装甲板，输出自动选择后的机器人的装甲兵版
         * @return: 一个RobotType
         */
        tdttoolkit::RobotType RobotDecide(std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor>> &input_armors);

        /**
         * @name: RobotDecide
         * @description: 半自动选择击打的机器人类型
         * @param: 输入分类且后的装甲板、需要击打的类型，输出半自动选择后的机器人的装甲兵版
         * @return: 一个RobotType
         */
        tdttoolkit::RobotType RobotDecide(std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor>> &input_armors,
                                          tdttoolkit::RobotType &type);

    private:
        float CommentArmors(tdttoolkit::ResolvedArmor &armor);

        tdttoolkit::ResolvedArmor Filter(std::vector<tdttoolkit::ResolvedArmor> &armors);//防止因装甲板的mark相近导致击打目标左右跳动

        void Sort();

        static bool CompareArmor( tdttoolkit::RobotArmor& armor_1, tdttoolkit::RobotArmor &armor_2);

    private:
        std::vector<std::pair<float,tdttoolkit::ResolvedArmor>> marks_;
        static float x_center_;
        static float y_center_;
        float mark_;
        bool comment_=false;//判断当前帧中两个装甲板在上一帧是否已经被同时评估；
        bool single_=true;//记录上一帧是否只有一个装甲板
        tdttoolkit::ResolvedArmor record_armor_;//保存其次的装甲板
        tdttoolkit::ResolvedArmor filter_armor_;//保存适合的装甲板

        float ratio_;
        float x_diff_;
        float y_diff_;
        float angle_;
        float area_;

    };
}

#endif //TDTVISION_RM2020_ROBOT_DECISION_H