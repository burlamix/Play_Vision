Original source of the data:
http://vasc.ri.cmu.edu/idb/html/motion/index.html

Point tracks courtesy of Rahul Raguram (raguram@cs.unc.edu).


The file measurement_matrix.txt contains 2d projections in the form 
of an 2M * N  matrix, where M is the number of views and N is the 
number of 3D points. The format is:

x1(1)  x2(1)   x3(1) ... xN(1)
y1(1)  y2(1)  y3(1) ... yN(1)
...
x1(M)  x2(M)   x3(M) ... xN(M)
y1(M)  y2(M)  y3(M) ... yN(M)

where xi(j) and yi(j) are the x- and y- projections of the ith 3D point 
in the jth camera. 

In total, there are 215 3D points visible in all 101 views.
The file movie.vmw visualizes the point tracks overlayed on
the image sequence.