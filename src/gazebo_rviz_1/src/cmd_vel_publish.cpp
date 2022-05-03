#include "cmd_vel_publish.h"
// ros::Rate rate(20);

bool CmdVel::AllPointsStoraged(const vector<geometry_msgs::Point> &points)
{
    int Size = points.size();
    if(Size<=10)
    {
        return false;
    }
    else
    {
        // cout<<points[Size].y<<" "<<points[Size-1].y<<endl;
        if(abs(points[Size-1].x - points[Size -2].x) < 0.005 && \
           abs(points[Size-1].y - points[Size -2].y) < 0.005 && \
           abs(points[Size-2].x - points[Size -3].x) < 0.005 && \
           abs(points[Size-2].y - points[Size -3].y) < 0.005
          )
        {
            isDown = true;
            // int num_partition = Size % 4 - 1;
            // vector<geometry_msgs::Point>::const_iterator it = points.begin();
            // points_1.assign(it,it+num_partition);
            // points_2.assign(it+num_partition+1, it+2*num_partition);
            // points_3.assign(it+2*num_partition+1, it+3*num_partition);
            // points_4.assign(it+3*num_partition+1, it+4*num_partition);
            // v_allpoints.push_back(&points_1);
            // v_allpoints.push_back(&points_2);
            // v_allpoints.push_back(&points_3);
            // v_allpoints.push_back(&points_4);
            // cout << "down111111" <<endl;
            return true;
        }
    }
    return false;
}

double TwoPointsAngleDifference(const double &x_curr, const double &y_curr, const double &x_goal, const double &y_goal)
{
    double ans = 0.0;
    if(y_goal<y_curr)
    {
        ans = atan2(y_goal-y_curr,x_goal-x_curr) + 2*3.14159;
    }
    else{
        ans = atan2(y_goal-y_curr,x_goal-x_curr);
    }
    return ans;
}

void CmdVel::YawOutputCalculation(vector<geometry_msgs::Point> &points)
{
    double yaw_;
    // vector<double> temp_v;
    for(int i=0;i<points.size()-1;++i)
    {
        yaw_ = TwoPointsAngleDifference(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        points[i].z = yaw_;

        //采样间隔10
        if(i%5 == 0 && i > 5)
        {
            double d_yaw = points[i].z - points[i-5].z;
            if(d_yaw<-4)
            {
                d_yaw+=2*pi;
            }
            if(d_yaw>4)
            {
                d_yaw-=2*pi;
            }
            d_yaw = d_yaw*2;
            v_yaw_output.push_back(d_yaw);
        }
    }
    for(int j=0;j<v_yaw_output.size();++j)
    {
        cout<<v_yaw_output[j]<<endl;
    }
    isDown_ = true;
}

double TheDistanceToGoalPoint(const double &x_curr, const double &y_curr, const double &x_goal, const double &y_goal)
{
    double ans = 0.0;
    ans = sqrt(pow(y_curr - y_goal,2) + pow(x_curr - x_goal,2));
    return ans;
}

double CmdVel::VelocityOutputCalculation(const vector<geometry_msgs::Point> &points, const int &index1, const int &index2)
{
    double bInterval = 0.0;
    for(int i=index1;i<index2;++i)
    {
        bInterval += TheDistanceToGoalPoint(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
    }
    return bInterval*2;
}

void *isArriveGoal(void *input)
{
    ros::Rate ra(2);
    CmdVel *cmd;
    cmd = (CmdVel *)input;
    int count = 0;
    // double bInterval = 0.0;
    while(ros::ok())
    {
        if(cmd->isDown)
        {
            if(!cmd->isDown_)
            {
                cmd->YawOutputCalculation(cmd->v_points);

                //计算两点之间的间隔的平均值
                // for(int i=0;i<cmd->v_points.size()-3;++i)
                // {
                //     bInterval += TheDistanceToGoalPoint(cmd->v_points[i].x, cmd->v_points[i].y, cmd->v_points[i+1].x, cmd->v_points[i+1].y);
                // }
                // bInterval = bInterval / (cmd->v_points.size() - 3);
                // cout<<"two point dis is:"<<bInterval<<endl;
                // cmd->v_out = bInterval*10;//速度是10倍的两点间隔，意味着1s走10个点
            }

            if(count<cmd->v_yaw_output.size())
            {
                cmd->yaw_out = cmd->v_yaw_output[count];
                int index1 = count * 5;
                int index2 = index1 + 5;
                cmd->v_out = cmd->VelocityOutputCalculation(cmd->v_points, index1, index2);
            }
            else{
                cmd->yaw_out = 0;
                cmd->v_out = 0;
            }
            count++;  
        } 
        ra.sleep();
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_publish");
    // ros::AsyncSpinner spinner(2);
    CmdVel Cmd;

    //在此处创建另一个（rosspin本身是一个阻塞式线程）线程，处理回调函数得到的位置信息，执行驱动算法
    pthread_t threads[1];
    int rec = pthread_create(&threads[0], NULL, isArriveGoal, (void *)&Cmd);
    if(rec){cout<<"ERROR:unable to create thread!"<<endl;}

    // rate.sleep();
    // spinner.start();
    ros::spin();
    return 0;
}