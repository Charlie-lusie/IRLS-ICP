/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_IRLS_SVD_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_IRLS_SVD_HPP_

#include <pcl/common/eigen.h>
#include <vector>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationIRLSSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  //estimateRigidTransformation (source_it, target_it, transformation_matrix, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimationIRLSSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  //estimateRigidTransformation (source_it, target_it, transformation_matrix, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimationIRLSSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  //estimateRigidTransformation (source_it, target_it, transformation_matrix, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  const int corr_size = correspondences.size();
  std::vector<float> distance, distance_sort;
  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> weights(4, corr_size);
  weights = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero(4, corr_size);
  distance.reserve(correspondences.size());
  distance_sort.reserve(correspondences.size());
  //weights.reserve(correspondences.size());
  for (int i = 0;i<correspondences.size();i++)
  {
	  int idx_q = correspondences[i].index_query, idx_m = correspondences[i].index_match;
	  float dis = sqrt((cloud_src.points[idx_q].x - cloud_tgt.points[idx_m].x)* (cloud_src.points[idx_q].x - cloud_tgt.points[idx_m].x) +
		  (cloud_src.points[idx_q].y - cloud_tgt.points[idx_m].y)*(cloud_src.points[idx_q].y - cloud_tgt.points[idx_m].y));
	  distance.push_back(dis);
	  distance_sort.push_back(dis);
  }
  std::sort(distance_sort.begin(), distance_sort.end(), std::greater<float>());
  int leng = distance_sort.size();
  float mid = 0.0;
  if (leng%2 == 0)
  {
	  mid = (distance_sort[leng / 2] + distance_sort[leng / 2 + 1]) / 2;
  }
  else
  {
	  mid = distance_sort[(int)(leng / 2) + 1];
  }
  mid *= 1.9;
  float max_dis = distance_sort[0];// ensure sort it from large to small
  if (mid < 1e-6*max_dis)
  {
	  mid = 0.3*max_dis;
  }
  else if (max_dis == 0)
	  mid = 1;
  mid = mid*4.7536 / 20;
  pcl::PointCloud<PointSource> cloud_src2;
  cloud_src2.width = cloud_src.width;
  cloud_src2.height = cloud_src.height;
  cloud_src2.is_dense = cloud_src.is_dense;
  cloud_src2.points.resize(cloud_src2.width * cloud_src2.height);
  for (int i = 0; i < weights.cols(); i++)
  {
	  //std::cout << correspondences[i].distance << std::endl;
	  //Here I found that distances in "correspondences" are not distances between corresoindent points
	  //They are some kinds of average distances or weights
	  weights(0, i) = (exp(-(distance[i] / mid)*(distance[i] / mid)));
	  weights(1, i) = weights(0, i);
	  weights(2, i) = weights(0, i);
	  cloud_src2.points[i].x = cloud_src.points[i].x * weights(0, i);// Here we compute weighted source data
	  cloud_src2.points[i].y = cloud_src.points[i].y * weights(0, i);
	  cloud_src2.points[i].z = cloud_src.points[i].z * weights(0, i);

  }

  ConstCloudIterator<PointSource> source_it(cloud_src2, correspondences, true);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);//false means using (correspondences.index_match)
  estimateRigidTransformation (source_it, target_it, transformation_matrix, weights);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    ConstCloudIterator<PointSource>& source_it,
    ConstCloudIterator<PointTarget>& target_it,
    Matrix4 &transformation_matrix,
	const Eigen::Matrix<Scalar, 4, Eigen::Dynamic> &weights) const
{
  // Convert to Eigen format
  const int npts = static_cast <int> (source_it.size ());

  if (use_umeyama_)
  {
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src (3, npts);
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt (3, npts);

    for (int i = 0; i < npts; ++i)
    {
      cloud_src (0, i) = source_it->x;
      cloud_src (1, i) = source_it->y;
      cloud_src (2, i) = source_it->z;
      ++source_it;

      cloud_tgt (0, i) = target_it->x;
      cloud_tgt (1, i) = target_it->y;
      cloud_tgt (2, i) = target_it->z;
      ++target_it;
    }

    // Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
    transformation_matrix = pcl::umeyama (cloud_src, cloud_tgt, false);
  }
  else
  {
    source_it.reset (); target_it.reset ();
    // <cloud_src,cloud_src> is the source dataset
    transformation_matrix.setIdentity ();

    Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
    // Estimate the centroids of source, target

    //compute3DCentroid (source_it, centroid_src);
    //compute3DCentroid (target_it, centroid_tgt);
    //source_it.reset (); target_it.reset ();//reset 指将指针指向数据开头

    // Subtract the centroids from source, target
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
	size_t npts = source_it.size(), npts2 = target_it.size();

	cloud_src_demean = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero(4, npts);        // keep the data aligned
	cloud_tgt_demean = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero(4, npts2);

	for (size_t i = 0; i < npts; ++i)
	{
		cloud_src_demean(0, i) = source_it->x;
		cloud_src_demean(1, i) = source_it->y;
		cloud_src_demean(2, i) = source_it->z;
		++source_it;
	}
	for (size_t i = 0; i < npts2; ++i)
	{
		cloud_tgt_demean(0, i) = target_it->x;
		cloud_tgt_demean(1, i) = target_it->y;
		cloud_tgt_demean(2, i) = target_it->z;
		++target_it;
	}
	source_it.reset(); target_it.reset();

	//demeanPointCloud (source_it, centroid_src, cloud_src_demean);
    //demeanPointCloud (target_it, centroid_tgt, cloud_tgt_demean);

    getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix, weights);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationIRLSSVD<PointSource, PointTarget, Scalar>::getTransformationFromCorrelation (
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_src_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_tgt,
    Matrix4 &transformation_matrix,
	const Eigen::Matrix<Scalar, 4, Eigen::Dynamic> &weights) const
{
  transformation_matrix.setIdentity ();
  Eigen::Vector4f med;//4*1
  Eigen::RowVector4f mem;//1*4
  med.setZero();
  mem.setZero();
  //std::cout << weights << std::endl;
  med = cloud_src_demean.rowwise().sum();
  for (int i = 0; i < cloud_src_demean.cols(); i++)//3*n
  {
	  /*med(0) += cloud_src_demean(0, i)*weights(0, i);
	  med(1) += cloud_src_demean(1, i)*weights(0, i);
	  med(2) += cloud_src_demean(2, i)*weights(0, i);*/
	  mem(0) += cloud_tgt_demean(0, i)*weights(0, i);
	  mem(1) += cloud_tgt_demean(1, i)*weights(0, i);
	  mem(2) += cloud_tgt_demean(2, i)*weights(0, i);
  }
  float sum_wei = weights.rowwise().sum()(0, 0);
  mem = mem / (sum_wei);//weight:4*n, we only need the sum of first row

  // Assemble the correlation matrix H = source * target'
  //std::cout << "src: "<< cloud_src_demean << std::endl;
  //std::cout << "tgt: " << cloud_tgt_demean.transpose() << std::endl;
  Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose() - med*mem).topLeftCorner(3, 3);
  //H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner (3, 3);

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU ();
  Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner (3, 3) = R;
  //const Eigen::Matrix<Scalar, 3, 1> Rc (R * centroid_src.head (3));
  //transformation_matrix.block (0, 3, 3, 1) = centroid_tgt.head (3) - Rc;
  const Eigen::Matrix<Scalar, 3, 1> Rc(R * (med / sum_wei).head(3));//4*4 4*1
  transformation_matrix.block(0, 3, 3, 1) = (mem.transpose()).head(3) - Rc;
}

//#define PCL_INSTANTIATE_TransformationEstimationSVD(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationSVD<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_HPP_ */
