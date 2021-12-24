# granular_rft_3D

This code introduces a method for implementing Granular Resistive Force Theory (RFT) in three dimensions. This code was developed in conjunction with the journal publication "Granular Resistive Force Theory Implementation for Three-Dimensional Trajectories," in IEEE Robotics and Automation Letters ( Volume: 6, Issue: 2 ) 03 February 2021. (DOI: 10.1109/LRA.2021.3057052).  https://ieeexplore.ieee.org/document/9345981


Laura Treers:  ltreers@berkeley.edu

Cyndia Cao: cyndia_cao@berkeley.edu

Hannah Stuart: hstuart@berkeley.edu

Embodied Dexterity Group, UC Berkeley Mechanical Engineering




*	MAIN shows two of the example bodies and trajectories that are explored in the paper, oscillation and circumnutation. 
	  * We use the function "stlread" to import meshes of the bodies that are exported from CAD models.
	      * “ellipsoid_exp.stl” and “root-tip.stl” are the stl files representing the ellipsoid body for oscillation and root tip paraboloid for circumnutation, respectively.
	      * This center of mass vector “stl_center” should be modified for each body geometry.
	  * The body geometry is processed and passed to the function that calculates the forces on the body, "RFT_3D_BODY".
	      * We utilize a RFT scaling coefficient of our glass beads of ζ=0.33; however, this coefficient should be calculated for each media.
	      * The oscillation and circumnutation trajectory parameters utilized mirror those introduced in the paper. These parameters should be tuned to individual systems, or different intrusion trajectories may be introduced by controlling the following inputs to RFT_3D_BODY: origin, orientation, vel, ang_vel.
	  * NOTE: we do not include a formulation for 2D RFT in this code. We have inserted some dummy functions to demonstrate the behavior of the code, but they are not tailored to any specific media. Users of this code should, for their own granular media, obtain coefficients α_x and α_z  as a function of intrusion angle and orientation. See "A Terradynamics of Legged Locomotion on Granular Media", Li et al, Science 339, 1408 (2013) for one example of 2D RFT implementation.
	
* RFT_3D_BODY Outputs the forces and torques on a body intruding through granular media.
   
    * Inputs: body geometry, body position & orientation, body velocity, RFT scaling coefficient (optional), STL representation (optional)
	 * Outputs: Force vector (N), Moment vector (N-cm)
	 * Note: All 3D position & velocity vectors are row vectors
	 * Note about angle definitions: Orientation of the elements are defined using 3-2-x Euler angles. Angular velocity is defined relative to body corotational basis.
* We request that all future works which utilize this code cite the publication "Granular Resistive Force Theory Implementation for Three-Dimensional Trajectories," in Robotics and Automation Letters 2021. (DOI: 10.1109/LRA.2021.3057052). Please feel free to reach out to the authors via email with any questions about implementing this code.
