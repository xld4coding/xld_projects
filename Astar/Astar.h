#pragma once

#include <vector>
#include <list>

const int kCost1 = 10;//直移一格消耗
const int kCost2 = 14;//斜移一格消耗

struct Point
{
    //对每一个点结构体，其包含的信息如下：
    int x,y;//坐标
    int F,G,H;//启发函数值
    Point *parent;//他的父节点
    //构造函数，FGH初始化为0
    Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL)
    {

    }
};

class Astar
{
public:
    //初始化
    void InitAstar(std::vector<std::vector<int>> &_maze);
    //主调函数，获取从start点到end点的路径
    std::list<Point *> GetPath(Point &startPoint,Point &endPoint,bool isIngnoreCorner);
private:
    //私有成员
    //寻找路径函数
    Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
    //获取当前Point周围的可用点
    std::vector<Point*>getSurroundPoints(const Point *point,bool isIgnoreCorner)const;
    //判断某点是否能从当前点到达
    bool isCanreach(const Point*point,const Point*target,bool isIgnoreCorner)const;//判断某点是否可以用于下一步判断
    //判断某点是否在开启/关闭列表中
    Point *isInList(const std::list<Point *>&list,const Point *point)const;
    //从开启列表中返回F值最小的节点
    Point *getLeastFpoint();
    //计算FGH值
    //计算G值
    int calcG(Point *temp_start,Point *point);
    //计算H值
    int calcH(Point *point,Point *end);
    //计算F值
    int calcF(Point *point);
private:
    //全局空间
    std::vector<std::vector<int>> maze;
    //开启列表
    std::list<Point *> openList;
    //关闭列表
    std::list<Point *> closelist;
};
