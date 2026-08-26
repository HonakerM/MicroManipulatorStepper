// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

//*** CLASS *****************************************************************************

class Pose6DF;

//--- IKinematicModel -------------------------------------------------------------------

class IKinematicModel {
  public:
    virtual ~IKinematicModel() {};

    // returns the number of joints
    virtual int get_joint_count() = 0;

    // computes the end effector pose from joint positions
    virtual bool foreward(const float* joint_positions, Pose6DF& pose) = 0;

    // computes joint positions from an end effector pose
    virtual bool inverse(const Pose6DF& pose, float* joint_positions) = 0;
};
