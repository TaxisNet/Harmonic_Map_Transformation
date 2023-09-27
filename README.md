# Harmonic_Map_Transformation

This matlab object is an implementation of [maxchaos' hntf2dsoftware package](https://github.com/maxchaos/hntf2d),
a harmonic transformation that maps any compact, multiply connected planar
domain onto the unit disk;
each of the holes that the original domain may have are collapsed onto
distinct points inside its image.

This transformation has been employed for addressing the navigation problem
of a single planar robot operating in arbitrarily shaped workspaces in 
[1](#org55c2b89), [2](#org6f2b086).

If you end up using this transformation and/or this software package,
please cite the aforementioned paper as follows:

    P. Vlantis, C. Vrohidis, C. P. Bechlioulis and K. J. Kyriakopoulos, "Robot Navigation in Complex Workspaces Using Harmonic Maps," 2018 IEEE International Conference on Robotics and Automation (ICRA), 2018, pp. 1726-1731, doi: 10.1109/ICRA.2018.8460695.

# Harmonic Map Class
HamronicMap is a matlab handle class. It was created in MATLAB R2021a. The constructor input is must be  1 x N cell each containing m x 2 points that define the boundaries. Boundary points must be in a counter clockwise order.
## Object Methods
### delete
delete	Delete a handle object.
### setBoundaries
recalculates the map
### map
maps point (x,y) to the 2x1 vector q is diskspace
### jacobian
returns the jacobian of point (x,y)
### compute
returns both the transform of points (x,y) and its jacobian
### plotMap
plots the workspace defined by boundaries and the disk space transform
### nagivate
A safe control scheme to navigate from any point [x_0; y_0]
towards a desired point [x_d;y_d] is simply calculated by 
[ux,uy]=-inv(J(p))*(f(p)-f(p_d)) for any p the belongs
to the interior of the workspace.

Outputs:
t is time vector,
p_path is the nx2 matrix of points in the workspace path that 
correspond to t 
and q_path is the nx2 matrix of the transformed points 
in diskspace.

Inputs: 
1. obj.navigation() makes you choose start and
destination points on the workspace plot (left).
2. obj.navigation(x_0,y_0,x_d,y_d) finds the paths without any
visual plots.
3. obj.navigate(x_0,y_0,x_d,y_d, vis) does the same as 2
but also visualizes in the map plot if vis is true.


# External Refrences 
1.  <a id="org55c2b89"></a> [scholar.google.com &#x2013; Robot navigation in complex workspaces using harmonic maps](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=R5c4qS8AAAAJ&citation_for_view=R5c4qS8AAAAJ:u-x6o8ySG0sC)
2.  <a id="org6f2b086"></a> [ieeexplore.ieee.org &#x2013; Robot Navigation in Complex Workspaces Using Harmonic Maps](https://ieeexplore.ieee.org/abstract/document/8460695)
