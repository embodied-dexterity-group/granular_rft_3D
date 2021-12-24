# granular_rft_3D

This code was developed in conjunction with the journal publication "Granular Resistive Force Theory Implementation for Three-Dimensional Trajectories," in IEEE Robotics and Automation Letters ( Volume: 6, Issue: 2 ) 03 February 2021. (DOI: 10.1109/LRA.2021.3057052).  https://ieeexplore.ieee.org/document/9345981


Laura Treers:  ltreers@berkeley.edu \
Cyndia Cao: cyndia_cao@berkeley.edu \
Hannah Stuart: hstuart@berkeley.edu \
Embodied Dexterity Group, UC Berkeley Mechanical Engineering



*	MAIN shows two of the example bodies and trajectories that are explored in the paper, oscillation and circumnutation. 
	  * We use the function "stlread" to import meshes of the bodies that are exported from CAD models.
	      * “ellipsoid-exp.stl” and “root-tip.stl” are the stl files representing the ellipsoid body for oscillation and root tip paraboloid for circumnutation, respectively.
	  * The body geometry is processed and passed to the function that calculates the forces on the body, "RFT_3D_BODY".
* RFT_3D_BODY Calculates the forces and torques on a body intruding through granular media.
    * Inputs: body geometry, body position & orientation, body velocity, RFT scaling coefficient (optional), STL representation (optional)
    * Outputs: Force vector (N), Moment vector (N-cm)
    * All 3D position & velocity vectors are row vectors.
    * The orientation of the elements are defined using 3-2-x Euler angles. Angular velocity is defined relative to body corotational basis.
    * NOTE: we do not include a formulation for 2D RFT in this code. We have inserted some stand-in functions to demonstrate the behavior of the code, but they are not tailored to any specific media. Users of this code should, for their own granular media, obtain coefficients α_x and α_z  as a function of intrusion angle and orientation. See "A Terradynamics of Legged Locomotion on Granular Media", Li et al, Science 339, 1408 (2013) for one example of 2D RFT implementation.
    	* Our glass beads have a RFT scaling coefficient of ζ=0.33; however, this coefficient should be measured for each media.
* We request that all future works which utilize this code cite the publication "Granular Resistive Force Theory Implementation for Three-Dimensional Trajectories," in Robotics and Automation Letters 2021. (DOI: 10.1109/LRA.2021.3057052). Please feel free to reach out to the authors via email with any questions about implementing this code.

## Code Output
These plots are produced by the released code. The force values are slightly different than the published paper, which uses Li et al's 2D RFT coefficients.

### Oscillation
<img src="https://user-images.githubusercontent.com/6529420/147366729-82c4e2b8-445d-4b1e-9ee8-9d29e10f3916.png" width="400"> <img src="https://user-images.githubusercontent.com/6529420/147366738-4827afa9-cbfc-472c-bc2e-1bf1e3a04119.png" width="400">

### Circumnutation
<img src="https://user-images.githubusercontent.com/6529420/147366747-d4299f6d-542c-400d-810c-f51c9b6fbd8e.png" width="400"> <img src="https://user-images.githubusercontent.com/6529420/147366749-144c2ee5-b2cf-414a-b24b-22cb7f6ccb44.png" width="400">

