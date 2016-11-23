#ifndef _LINEAR_COMPARISON_H
#define _LINEAR_COMPARISON_H

#include <cassert>
#include <pcl/filters/conditional_removal.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::LinearComparison<PointT>::LinearComparison (
    std::string field_name_a, ComparisonOps::CompareOp op,
    double scale_val, std::string field_name_b)
  : ComparisonBase<PointT> ()
  , scale_val_ (scale_val), point_data_ (NULL)
{
  field_name_a_ = field_name_a;
  op_ = op;
  scale_val_ = scale_val;
  field_name_b_ = field_name_b;

  // Get all fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Find field_name
  if (point_fields.empty ())
  {
    PCL_WARN ("[pcl::LinearComparison::LinearComparison] no fields found!\n");
    capable_ = false;
    return;
  }

  // Get the field index
  
  size_t d_a;
  for (d_a = 0; d_a < point_fields.size (); ++d_a)
  {
    if (point_fields[d_a].name == field_name_a) 
      break;
  }
  if (d_a == point_fields.size ())
  {
    PCL_WARN ("[pcl::LinearComparison::LinearComparison] field 0 not found!\n");
    capable_ = false;
    return;
  }

  size_t d_b;
  for (d_b = 0; d_b < point_fields.size (); ++d_b)
  {
    if (point_fields[d_b].name == field_name_b) 
      break;
  }
  if (d_b == point_fields.size ())
  {
    PCL_WARN ("[pcl::LinearComparison::LinearComparison] field 1 not found!\n");
    capable_ = false;
    return;
  }
  
  assert(point_fields[d_a].datatype == point_fields[d_b].datatype);
  uint8_t datatype = point_fields[d_a].datatype;
  uint32_t offset_a = point_fields[d_a].offset;
  uint32_t offset_b = point_fields[d_b].offset;


  capable_ = true;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::LinearComparison<PointT>::~LinearComparison () 
{
  if (point_data_ != NULL)
  {
    delete point_data_;
    point_data_ = NULL;
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::LinearComparison<PointT>::evaluate (const PointT &point) const
{
  if (!this->capable_)
  {
    PCL_WARN ("[pcl::LinearComparison::evaluate] invalid compariosn!\n");
    return (false);
  }

  // if p(data) > val then compare_result = 1
  // if p(data) == val then compare_result = 0
  // if p(data) <  ival then compare_result = -1
  const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&point);

  
  switch (datatype) 
  {
    case pcl::PCLPointField::INT8 :
    {
      int8_t pt_val_a;
      int8_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (int8_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (int8_t));
      return (pt_val_a > static_cast<int8_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<int8_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::UINT8 :
    {
      uint8_t pt_val_a;
      uint8_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (uint8_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (uint8_t));
      return (pt_val_a > static_cast<uint8_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<uint8_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::INT16 :
    {
      int16_t pt_val_a;
      int16_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (int16_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (int16_t));
      return (pt_val_a > static_cast<int16_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<int16_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::UINT16 :
    {
      uint16_t pt_val_a;
      uint16_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (uint16_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (uint16_t));
      return (pt_val_a > static_cast<uint16_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<uint16_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::INT32 :
    {
      int32_t pt_val_a;
      int32_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (int32_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (int32_t));
      return (pt_val_a > static_cast<int32_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<int32_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::UINT32 :
    {
      uint32_t pt_val_a;
      uint32_t pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (uint32_t));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (uint32_t));
      return (pt_val_a > static_cast<uint32_t> (scale_val * pt_val_b)) - (pt_val_a < static_cast<uint32_t> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::FLOAT32 :
    {
      float pt_val_a;
      float pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (float));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (float));
      return (pt_val_a > static_cast<float> (scale_val * pt_val_b)) - (pt_val_a < static_cast<float> (scale_val * pt_val_b));
    }
    case pcl::PCLPointField::FLOAT64 :
    {
      double pt_val_a;
      double pt_val_b;
      memcpy (&pt_val_a, pt_data + this->offset_a, sizeof (double));
      memcpy (&pt_val_b, pt_data + this->offset_b, sizeof (double));
      return (pt_val_a > scale_val * pt_val_b) - (pt_val_a < scale_val * pt_val_b);
    }
    default : 
      PCL_WARN ("[pcl::LinearComparison::evaluate] unknown data_type!\n");
      return (false);
  }
  
  switch (this->op_)
  {
    case pcl::ComparisonOps::GT :
      return (compare_result > 0);
    case pcl::ComparisonOps::GE :
      return (compare_result >= 0);
    case pcl::ComparisonOps::LT :
      return (compare_result < 0);
    case pcl::ComparisonOps::LE :
      return (compare_result <= 0);
    case pcl::ComparisonOps::EQ :
      return (compare_result == 0);
    default:
      PCL_WARN ("[pcl::LinearComparison::evaluate] unrecognized op_!\n");
      return (false);
  }
}


#endif
