#include <math.h>
#include "Astar.h"

//给Astar类私有成员maze赋值，创建全局空间
void Astar::InitAstar(std::vector<std::vector<int>>&_maze)
{
    maze=_maze;
}
//1
int Astar::calcG(Point *temp_start,Point*point)
{
    int extraG=(abs(point->x-temp_start->x))+abs(point->y-temp_start->y)==1?kCost1:kCost2;
    int parentG=point->parent==NULL?0:point->parent->G;//如果是初始节点，则其父节点是空
    return parentG +extraG;//G值等于其父节点的G值加上父节点转移到该节点增加的G值
}
//2
int Astar::calcH(Point*point,Point *end)
{
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没有深入研究--
    return sqrt((double)(end->x-point->x)*(double)(end->x-point->x)+(double)(end->y-point->y)*(double)(end->y-point->y))*kCost1;
}
//3
int Astar::calcF(Point*point)
{
    return point->G+point->H;//F=G+H
}
//4
Point *Astar::getLeastFpoint()
{
    if(!openList.empty())
    {
        auto resPoint =openList.front();//取头个节点
        for(auto &point:openList)//遍历列表
        if(point->F<resPoint->F)//如果遍历到的节点F值小于resPoint的F
           resPoint=point;//更新resPoint
        return resPoint;//返回开放列表中最小F值的节点
    }
    return NULL;//如果开放列表为空，返回NULL
}
//5
Point *Astar::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
    //置入起点，拷贝开辟一个节点，内外隔离?
    openList.push_back(new Point(startPoint.x,startPoint.y));
    while(!openList.empty())
    {
        auto curPoint = getLeastFpoint();//找到openList中F值最小的点
        openList.remove(curPoint);//将其从开启列表中删除
        closelist.push_back(curPoint);//
        //1,找到当前格curPoint周围八个格中可以通过的节点
        auto surroundPoints =getSurroundPoints(curPoint,isIgnoreCorner);
        //遍历周围节点
        for(auto &target:surroundPoints)
        {
            //2，对curPoint周围某一个格子，如果它不在开启列表中，则将其加入开启列表，设置当前格子为其父节点，计算他的FGH
            if(!isInList(openList,target))
            {
                target->parent=curPoint;

                target->G=calcG(curPoint,target);
                target->H=calcH(target,&endPoint);
                target->F=calcF(target);

                openList.push_back(target);
            }
            //3，对curPoint周围某一个格子，如果它已经在开启列表中了，则计算他的G值，如果比原来的大，就什么都不做，否则设置它的父节点为curPoint，并更新G和F
            else
            {
                //计算curPoint到target的G值
                int tempG=calcG(curPoint,target);
                if(tempG<target->G)
                {
                    target->parent=curPoint;

                    target->G=tempG;
                    target->F=calcF(target);
                }
            }
            Point *resPoint=isInList(openList,&endPoint);
            if(resPoint)
               return resPoint;//返回列表里的节点指针，不要用原来传入的enpoint,因为发生了深拷贝
            
        }
    }
    return NULL;
}
//6
std::list<Point *> Astar::GetPath(Point &startPoint,Point &endPoint,bool isIngnoreCorner)
{
    Point*result=findPath(startPoint,endPoint,isIngnoreCorner);
    std::list<Point *> path;
    //
    while(result)
    {
        path.push_front(result);
        result=result->parent;
    }
    //
    openList.clear();
    closelist.clear();

    return path;
}
//7
Point *Astar::isInList(const std::list<Point*>&list,const Point*point)const
{
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
    for(auto p:list)
    if(p->x==point->x&&p->y==point->y){
        return p;
    }
    return NULL;
}
//8
bool Astar::isCanreach(const Point*point,const Point*target,bool isIgnoreCorner)const
{
    if(target->x<0||target->x>maze.size()-1
       ||target->y<0||target->y>maze[0].size()-1
       ||maze[target->x][target->y]==1
       ||target->x == point->x&&target->y == point->y
       ||isInList(closelist, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
       return false;
       else
       {
           //非斜角可以
           if(abs(point->x-target->x)+abs(point->y-target->y)==1)
           return true;
           else
           {
               //斜对角要判断是否绊住?
               if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
               return true;
               else
               {
                   return isIgnoreCorner;
               }
               
           }
           
       }
       
}
//9
std::vector<Point *>Astar::getSurroundPoints(const Point *point,bool isIgnoreCorner)const
{
    std::vector<Point *> surroundPoints;
    for (int x = point->x - 1; x <= point->x + 1; x++)
    for (int y = point->y - 1; y <= point->y + 1; y++)
    if(isCanreach(point,new Point(x,y),isIgnoreCorner))
      surroundPoints.push_back(new Point(x,y));
    return surroundPoints;
}