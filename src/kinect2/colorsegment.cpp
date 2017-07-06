#include <fstream>
#include "colorsegment.h"
#include <iostream>

nubot::ColorSegment::ColorSegment(const char*  _table_name)
{
    std::ifstream CTable_Read(_table_name, std::ios::binary|std::ios::in);//以输入方式打开文件;ios::binary：以二进制方式打开文件，缺省的方式是文本方式。ios::in：文件以输入方式打开（文件数据输入到内存）
    CTable_Read.read((char *)table_,sizeof(unsigned char)*64*64*64);//从文件中读取 num 个字符到 buf 指向的缓存中;读写二进制数据块，使用成员函数read()和write()成员函数
    CTable_Read.close();
}

int
nubot::ColorSegment::Segment(cv::Mat &_image, const cv::Rect &_ROI)
{
  if(_image.cols==0 || _image.rows==0)
    return(-1);
  else if(_image.cols!=segment_result_.cols || _image.rows!=segment_result_.rows)
  {
    check_flag_.create(_image.rows,_image.cols,CV_8UC1);//???
    segment_result_.create(_image.rows,_image.cols,CV_8UC1);
  }
  if(_image.channels()<3)
    return(-1);
  int Xmin = _ROI.x;
  int Ymin = _ROI.y;
  int Xmax = _ROI.x+_ROI.width;
  int Ymax = _ROI.y+_ROI.height;
  uchar*	ptr_result = segment_result_.data;//segment_result_是个cv::Mat
  uchar*	ptr_image = _image.data;
  int num=0;
  for(int i=0; i<_image.rows; i++)
  {
    ptr_result = segment_result_.data+segment_result_.step[0]*i;//Mat中一个uchar* data指向矩阵数据的首地址;step[0]，step[1]表示每一行和每一个元素的数据大小
    ptr_image  = _image.data+_image.step[0]*i;
    for(int j=0; j<_image.cols; j++)
    {
      if((Ymin<=i)&&(i<=Ymax)&&(Xmin<=j)&&(j<=Xmax))
      {
        *ptr_result = table_[ptr_image[0]/4*64*64 + ptr_image[1]/4*64 + ptr_image[2]/4];//???
        if(*ptr_result==0)
            num+=1;
      }
      else
        *ptr_result = VISION_COLORSEGMENT_UNKNOWCOLOR;
      ptr_result += segment_result_.step[1];//指向下一个元素
      ptr_image += _image.step[1];
    }
  }
  return(num);
}

bool
nubot::ColorSegment::RegionSearch(std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas, const cv::Rect &_ROI, const int &_threshold_size, const int &_threshold_combination, unsigned char _target_color)
{
  if(check_flag_.cols!=segment_result_.cols || check_flag_.rows!=segment_result_.rows)
    return(false);
  if(segment_result_.rows*segment_result_.cols<_threshold_size)
    return(false);
  if((double)(_ROI.width)*_ROI.height<_threshold_size)
    return(false);

  //some initializations
  static const cv::Point2i neighbors[8]={cv::Point(-1,-1),cv::Point(0,-1),cv::Point(1,-1),
                                         cv::Point(-1,0),                 cv::Point(1, 0),
                                         cv::Point(-1, 1),cv::Point(0, 1),cv::Point(1, 1)};
  ImageArea current_area;//the current area
  long x_sum, y_sum;//used to calculate the centroid of current area
  int area_rect_Xmin;//the ex-rectangle of current area
  int area_rect_Ymin;//the ex-rectangle of current area
  int area_rect_Xmax;//the ex-rectangle of current area
  int area_rect_Ymax;//the ex-rectangle of current area
  cv::Point current_point;//the current point when processing
  cv::Point neighbor_point;//the neighbor of current point when region grow
  int neighbor_amount;//count the neighbors of current point. if less than 8, current point is edge point

  check_flag_.setTo(0);
  int Xmin = _ROI.x;//can be equal
  int Ymin = _ROI.y;//can be equal
  int Xmax = _ROI.x+_ROI.width;//can NOT be equal
  int Ymax = _ROI.y+_ROI.height;//can NOT be equal
  Xmin = cv::max(0,cv::min(segment_result_.cols,Xmin));
  Xmax = cv::max(0,cv::min(segment_result_.cols,Xmax));
  Ymin = cv::max(0,cv::min(segment_result_.rows,Ymin));
  Ymax = cv::max(0,cv::min(segment_result_.rows,Ymax));
  std::vector<ImageArea> area_list;//once a candidate area is found, it will be pushed in this list

  //start process
  //1. search areas no mater how small they are
  for(int y=Ymin; y<Ymax; y++)
  {
    for(int x=Xmin; x<Xmax; x++)
    {
      current_point = cv::Point(x,y);
      //for every pixel, if it is _target_color, it must be put in an area. the check_flag_ mat marked if a pixel has already been put in a area.
      if(segment_result_.at<unsigned char>(current_point)==_target_color && !check_flag_.at<unsigned char>(current_point))
      {
        //if a pixel is _target_color, but has not been put in an area, a new area is established to take this pixel in.  then find all pixels which is connected with this area and put them in.
        check_flag_.at<unsigned char>(current_point) = true;
        grow_queue_.push_back(current_point);
        x_sum = 0;
        y_sum = 0;
        current_area.edge_points_.clear();
        current_area.area_size_ = 0;
        area_rect_Xmin = area_rect_Xmax = current_point.x;
        area_rect_Ymin = area_rect_Ymax = current_point.y;
        //start to search pixels which is connected with this area and put them in
        while(grow_queue_.size()>0)
        {
          current_point = grow_queue_.front();
          x_sum += current_point.x;
          y_sum += current_point.y;
          current_area.area_size_++;
          neighbor_amount = 0;
          for(int k=0; k<8; k++)
          {
            neighbor_point = current_point+neighbors[k];
            if(neighbor_point.x<Xmax && neighbor_point.x>=Xmin && neighbor_point.y<Ymax && neighbor_point.y>Ymin && segment_result_.at<unsigned char>(neighbor_point)==_target_color)
            {
              neighbor_amount++;
              if(check_flag_.at<unsigned char>(neighbor_point)==false)
              {
                check_flag_.at<unsigned char>(neighbor_point) = true;
                grow_queue_.push_back(neighbor_point);
              }
            }
          }
          if(neighbor_amount<8)
          {
            //storage edge points of current area
            current_area.edge_points_.push_back(current_point);
            //update area rect of current area
            if(current_point.x<area_rect_Xmin)
              area_rect_Xmin = current_point.x;
            else if(current_point.x>area_rect_Xmax)
              area_rect_Xmax = current_point.x;
            if(current_point.y<area_rect_Ymin)
              area_rect_Ymin = current_point.y;
            else if(current_point.y>area_rect_Ymax)
              area_rect_Ymax = current_point.y;
          }
          grow_queue_.pop_front();
        }
        //one area is complete
        if(current_area.area_size_>=_threshold_size)
        {
          current_area.area_center_ = cv::Point(x_sum/current_area.area_size_,y_sum/current_area.area_size_);
          current_area.area_rect_ = cv::Rect(area_rect_Xmin,area_rect_Ymin,area_rect_Xmax-area_rect_Xmin,area_rect_Ymax-area_rect_Ymin);
          area_list.push_back(current_area);
        }

      }//end of processing one area
    }//for(int x=Xmin; x<Xmax; x++)
  }//for(int y=Ymin; y<Ymax; y++)

  if(area_list.size()==0)
    return(false);

  //2. combine areas nearby
  if(_threshold_combination>0)
  {
    
  }

  //3. rank the areas by size
  std::vector<ImageArea>::iterator temp_area_iterator;
  std::vector<ImageArea>::iterator current_area_iterator;
  _target_areas.clear();
  for(int i=0; i<_max_num_of_areas; i++)
  {
    if(area_list.size()==0)
      break;
    //search the max size area in area_list and put it in _target_areas.
    current_area_iterator = area_list.begin();
    for(temp_area_iterator=area_list.begin(); temp_area_iterator!=area_list.end(); temp_area_iterator++)
    {
      if(temp_area_iterator->area_size_>current_area_iterator->area_size_)
        current_area_iterator = temp_area_iterator;
    }
    _target_areas.push_back(*current_area_iterator);
    area_list.erase(current_area_iterator);
  }
  return(true);
}

bool
nubot::ColorSegment::RegionSearch(ImageArea &_target_area)
{
  std::vector<ImageArea> areas;
  if(!RegionSearch(areas))
    return(false);
  _target_area = *areas.begin();
  return(true);
}
