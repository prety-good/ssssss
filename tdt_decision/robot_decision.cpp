/*
 * @Name: Decision
 * @Description: 击打决策
 * @Version: 1.0.0.0
 * @Author: 杨泽旭
 * @Date: 2020-2-14
 * @LastEditors:
 * @LastEditTime:
 */
#include "robot_decision.h"
namespace tdtdecision {
    float RobotDecision::x_center_;
    float RobotDecision::y_center_;
    RobotDecision::RobotDecision() {
        float temp_x;
        LoadParam::ReadTheParam("SrcWidth", temp_x);
        x_center_ = temp_x / 2;
        float temp_y;
        LoadParam::ReadTheParam("SrcHeight", temp_y);
        y_center_ = temp_y / 2;
        mark_ = 0;

        ratio_ = 0;
        x_diff_ = 0;
        y_diff_ = 0;
        angle_ = 0;
        area_ = 0;

    }

    tdttoolkit::ResolvedArmor RobotDecision::ArmorDecision(std::vector<tdttoolkit::ResolvedArmor> &armors) {
        if (armors.empty()) {
            comment_ = false;
            TDT_ERROR("TDT_DECISION: The vector of the armors inputted is empty!");
        } else if (armors.size() == 1) {
            comment_ = false;
            if (single_) {
//                    std::cout<<"single=true"<<std::endl;
                filter_armor_ = armors[0];
                return armors[0];
            } else {
                float x = armors[0].GetStickerRect().GetCenter2f().x;
                float y = armors[0].GetStickerRect().GetCenter2f().y;
                if(armors[0].GetRobotType()!=filter_armor_.GetRobotType()){
                    TDT_INFO("single=false,robot type is different,use filter armor");
                    single_ = true;
                    return filter_armor_;
                }
                else if (x > (record_armor_.GetStickerRect().GetCenter2f().x - 150) &&
                         x < (record_armor_.GetStickerRect().GetCenter2f().x + 150)) {
                    if (y > (record_armor_.GetStickerRect().GetCenter2f().y - 150) &&
                        y < (record_armor_.GetStickerRect().GetCenter2f().y + 150)) {
//                     std::cout<<"single=false,use filter armor"<<std::endl;
                        record_armor_ = armors[0];
                        single_ = true;
                        return filter_armor_;
                    }
//                 std::cout<<"single=false,use latest armor"<<std::endl;
                    filter_armor_ = armors[0];
                    single_ = true;
                    return armors[0];
                }
                else {
//                 std::cout<<"single=false 2,use latest armor"<<std::endl;
                    filter_armor_ = armors[0];
                    single_ = true;
                    return armors[0];
                }
            }

        } else {
            if(armors.size()>3){
                for(int i=2;i<armors.size();i++){
                    armors.erase(armors.begin()+i);
                }
            }
            //以下部分皆在”已经锁定了最优打击目标“和“已经识别出此目标上的两个装甲板”的条件下成立
            if(armors[0].GetRobotType()!=filter_armor_.GetRobotType() && !single_){
                TDT_INFO("single=false,robot type is different,use filter armor");
                single_ = true;
                return filter_armor_;
            }
            if (comment_) {
                marks_.clear();
                for (int i = 0; i < armors.size(); i++) {
//                 std::cout<<"---------------------"<<i<<"----------------------"<<std::endl;
                    float temp_mark = CommentArmors(armors[i]);
//                 std::cout<<"*mark:"<<temp_mark<<std::endl;
//                 std::cout<<"--------------------------------------------"<<std::endl;
                    std::pair<float, tdttoolkit::ResolvedArmor> temp_pair(temp_mark, armors[i]);
                    marks_.push_back(temp_pair);
                }
                Sort();
                single_ = false;

                return Filter(armors);
            } else {
                marks_.clear();
                for (int i = 0; i < armors.size(); i++) {
//                 std::cout<<"---------------------"<<i<<"----------------------"<<std::endl;
                    float temp_mark = CommentArmors(armors[i]);
//                 std::cout<<"*mark:"<<temp_mark<<std::endl;
//                 std::cout<<"--------------------------------------------"<<std::endl;
                    std::pair<float, tdttoolkit::ResolvedArmor> temp_pair(temp_mark, armors[i]);
                    marks_.push_back(temp_pair);
                }
                Sort();//分数从大到小排序
                comment_ = true;
                single_ = false;
                record_armor_ = marks_[1].second;
                filter_armor_ = marks_[0].second;
                return marks_[0].second;
            }
        }
    }

    void RobotDecision::Sort() {
        for (int i = 0; i < marks_.size() - 1; i++) {
            for (int j = i + 1; j < marks_.size(); j++) {
                if (marks_[i].first < marks_[j].first) {
                    auto temp = marks_[j];
                    marks_[j] = marks_[i];
                    marks_[i] = temp;
                }
            }
        }
    }

    float RobotDecision::CommentArmors(tdttoolkit::ResolvedArmor &armor) {
        mark_ = 0;

        angle_ = armor.GetStickerRect().GetAngle();
        area_ = armor.GetStickerRect().GetHeight() * armor.GetStickerRect().GetWidth();
        ratio_ = armor.GetStickerRect().GetHeight() / armor.GetStickerRect().GetWidth();
        x_diff_ = fabs(armor.GetStickerRect().GetCenter2f().x - x_center_);
        y_diff_ = fabs(armor.GetStickerRect().GetCenter2f().y - y_center_);
//    std::cout<<"*ratio:"<<ratio_<<std::endl;
//    std::cout<<"*x_diff:"<<x_diff_<<std::endl;
//    std::cout<<"*y_diff:"<<y_diff_<<std::endl;
//    std::cout<<"*angle:"<<angle_<<std::endl;
//    std::cout<<"*area:"<<area_<<std::endl;

        mark_ =- angle_ * 5.12 + area_ * 0.0342 + ratio_ * 312 - x_diff_ * 0.56 - y_diff_ ;
        // std::cout << "mark_" << mark_<<std::endl;
        comment_ = true;
        return mark_;
    }

    tdttoolkit::ResolvedArmor RobotDecision::Filter(std::vector<tdttoolkit::ResolvedArmor> &armors) {
        if (fabs(marks_[0].first - marks_[1].first) < 120) {
            for (int i = 0; i < armors.size(); i++) {
                float x = armors[i].GetStickerRect().GetCenter2f().x;
                float y = armors[i].GetStickerRect().GetCenter2f().y;
                if (x > (filter_armor_.GetStickerRect().GetCenter2f().x - 150) &&
                    x < (filter_armor_.GetStickerRect().GetCenter2f().x + 150)) {
                    if (y > (filter_armor_.GetStickerRect().GetCenter2f().y - 150) &&
                        y < (filter_armor_.GetStickerRect().GetCenter2f().y + 150)) {
                        filter_armor_ = armors[i];
                        record_armor_ = armors[1 - i];
                        comment_ = true;
//                    std::cout<<"similar"<<std::endl;
                        return armors[i];
                    }
                }
            }
//        std::cout<<"not similar"<<std::endl;
            filter_armor_ = marks_[0].second;
            record_armor_ = marks_[1].second;
            comment_ = true;
            return marks_[0].second;
        } else {
//        std::cout<<"not similar"<<std::endl;
            filter_armor_ = marks_[0].second;
            record_armor_ = marks_[1].second;
            comment_ = true;
            return marks_[0].second;
        }
    }

    std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor>> RobotDecision::ClassifyArmors(std::vector<tdttoolkit::RobotArmor> &input_armors) {
        std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor> > output_armors;
        if(input_armors.empty()){
            std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor> >().swap(output_armors);
            return output_armors;
        }
        else{
            for(auto & input_armor : input_armors){
                if(input_armor.GetRobotType()==tdttoolkit::RobotType::TYPEUNKNOW){
                    continue;
                }
                else{
                    output_armors[input_armor.GetRobotType()].push_back(input_armor);
                    std::cout<<input_armor.GetRobotType()<<std::endl;
                }
            }
            return output_armors;
        }
    }

    tdttoolkit::RobotType RobotDecision::RobotDecide(std::map<tdttoolkit::RobotType,std::vector<tdttoolkit::RobotArmor>> &input_armors) {
        if(input_armors.empty()){
            std::vector<tdttoolkit::RobotArmor>().swap(input_armors[tdttoolkit::TYPEUNKNOW]);
            return tdttoolkit::TYPEUNKNOW;
        }
        if(input_armors.size()==1){
            std::cout<<"自动选择：只有一种敌方车"<<std::endl;
            return input_armors.begin()->first;
        }
        else{
            std::vector<tdttoolkit::RobotArmor> temp_armors;
            for(auto &it : input_armors){
                std::sort(it.second.begin(),it.second.end(),CompareArmor);
                temp_armors.push_back(*it.second.begin());
                std::cout<<"机器人"<<it.second[0].GetRobotType()<<"离中心最近的装甲板："<<it.second[0].GetStickerRect().GetCenter()<<std::endl;
            }
            std::sort(temp_armors.begin(),temp_armors.end(),CompareArmor);
            std::cout<<"离中心最近的机器人的离中心最近最近的装甲板："<<temp_armors[0].GetStickerRect().GetCenter()<<std::endl;
            if(temp_armors.begin()->GetRobotType()!=tdttoolkit::RobotType::ENGINEER)return temp_armors.begin()->GetRobotType();
            else {
                return temp_armors[1].GetRobotType();
            }
        }
    }

    tdttoolkit::RobotType RobotDecision::RobotDecide(std::map<tdttoolkit::RobotType, std::vector<tdttoolkit::RobotArmor>> &input_armors,
                                                     tdttoolkit::RobotType &type) {
        if(input_armors.find(type)==input_armors.end()){
            return RobotDecide(input_armors);
        }
        else {
            return type;
        }
    }

    bool RobotDecision::CompareArmor( tdttoolkit::RobotArmor& armor_1, tdttoolkit::RobotArmor &armor_2) {
        return (tdttoolkit::CalcDistance(armor_1.GetStickerRect().GetCenter(),cv::Point(x_center_,y_center_))
                <tdttoolkit::CalcDistance(armor_2.GetStickerRect().GetCenter(),cv::Point(x_center_,y_center_)));
    }

}