import open3d as o3d 
import copy
import numpy as np 
import math
import math


pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

pcd= np.asarray(pcd.points)   #Converting the point cloud file as an array


Iterations = 1500

Threshold = 0.1

for j in range(Iterations):    
    Rand= np.random.randint(0, len(pcd)-1, size=3)
    

    p1= pcd[Rand[0]]    
    p2= pcd[Rand[1]]
    p3= pcd[Rand[2]]
    

    x1,y1,z1= p1[0],p1[1],p1[2]  
    x2,y2,z2= p2[0],p2[1],p2[2]
    x3,y3,z3= p3[0],p3[1],p3[2]
    
    a= (((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1)))  
    b= (((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1)))
    c= (((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1)))
    d= - ((a*x1) + (b*y1) + (c*z1))
    

   
    distance= np.abs(((a*pcd[:,0]) + (b*pcd[:,1]) + (c*pcd[:,2]) + d)/(math.sqrt(np.abs((a*a) + (b*b) + (c*c)))))
    distance= np.round(distance, 2)
    
    plane_parameters= [a,b,c,d]
    
    index=([])
    inliers=([])
    most_inliers=([])
    
    
    i=0 
    for i in range(len(distance)):
        if distance[i] <= Threshold :
            index.append(i)
    
    inliers= pcd[index]
    
    
    if len(inliers)> len(most_inliers):

        plane_coefficients= plane_parameters
        best_inliers= inliers
        best_inliers_index= index


print(" plane Equation  %s x + %s y + %s z + %s "
         %(plane_coefficients[0],plane_coefficients[1],plane_coefficients[2],plane_coefficients[3]))





best_inliers=np.concatenate(best_inliers, axis=0)

pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

inliers_array= pcd.select_by_index(np.asarray(best_inliers_index))
outlier_cloud = pcd.select_by_index(np.asarray(best_inliers_index), invert=True)


inliers_array.paint_uniform_color([1.0, 1.0, 0])



o3d.visualization.draw_geometries([inliers_array, outlier_cloud],
                                  zoom=0.8,
                                front=[-0.4999, -0.1659, -0.8499],
                               lookat=[2.1813, 2.0619, 2.0999],up=[0.1204, -0.9852, 0.1215])