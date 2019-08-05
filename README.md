# IRLS-ICP
An IRLS-ICP implementation based on PCL
 
##Usage
You can directly replace **"registration"** folder in path: **"your-path\PCL 1.8.0\include\pcl-1.8\pcl\registration"** of your PCL path. Example code can be like this:

```C++
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.setMaxCorrespondenceDistance(5);
	icp.setTransformationEpsilon(1e-1);
	icp.setEuclideanFitnessEpsilon(5);
	icp.setMaximumIterations(20);
	typedef pcl::registration::TransformationEstimationIRLSSVD<pcl::PointXYZ, pcl::PointXYZ> SVD2;
	boost::shared_ptr<SVD2> trans_svd(new SVD2(false));
	icp.setTransformationEstimation(trans_svd);
	icp.align(Final);
	transform = icp.getFinalTransformation();
```
## Details
Welsh weight function is used in this implementation, which is:
$$w = e^{-(\frac{x}{c})^2}$$
Where c is suggested as $c = \lambda (1.9*median(dis)),\lambda=4.7536$


##Reference

- BergstroM, Per, and O. Edlund. "Robust registration of point sets using iteratively reweighted least squares." Computational Optimization and Applications 58.3(2014):543-561.
- Zhengyou Zhang. "Parameter estimation techniques: a tutorial with application to conic fitting".

