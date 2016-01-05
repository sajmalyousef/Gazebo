/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "CustomDiffDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CustomDiffDrivePlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
CustomDiffDrivePlugin::CustomDiffDrivePlugin()
{
  this->wheelSeparation = .100;   //though given here, it will be calculated from the model in Init()
  this->wheelRadius = .016;       //though given here, it will be calculated from the model in Init()

}

/////////////////////////////////////////////////
void CustomDiffDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->TargetPosSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/targetpos_cmd", &CustomDiffDrivePlugin::OnTargetPosMsg, this);

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";

    if (!_sdf->HasElement("acrylic_base"))
    gzerr << "DiffDrive plugin missing <acrylic_base> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->Get<std::string>());
  this->acrylicBase = _model->GetLink(
      _sdf->GetElement("acrylic_base")->Get<std::string>());

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";
  if (!this->acrylicBase)
    gzerr << "Unable to find acrylicbase["
          << _sdf->GetElement("acrylic_base")->Get<std::string>() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CustomDiffDrivePlugin::OnUpdate, this));

}

/////////////////////////////////////////////////
void CustomDiffDrivePlugin::Init()
{
  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;


  this->leftJoint->SetVelocityLimit(0, .1/this->wheelRadius);
  this->rightJoint->SetVelocityLimit(0, .1/this->wheelRadius);
}

/////////////////////////////////////////////////
void CustomDiffDrivePlugin::OnTargetPosMsg(ConstPosePtr &_msg)
{
  double xtarget = -1, ytarget = 10;// thetaTarget;  --> Not implimented

  xtarget = _msg->position().x();
  ytarget = _msg->position().y();

}

/////////////////////////////////////////////////
void CustomDiffDrivePlugin::OnUpdate()
{

  double x = acrylicBase->GetWorldPose().pos.x;
  double y = acrylicBase->GetWorldPose().pos.y;
  double theta = acrylicBase->GetWorldPose().pos.y;

  if ((x - xtarget)*(x - xtarget) + (y - ytarget)*(y - ytarget) < .011)
  {
    this->leftJoint->SetVelocity(0, 0);
    this->rightJoint->SetVelocity(0, 0);
  }
  else
  {
    float tmpTheta = theta;
    if(theta > 3.14)
      tmpTheta = 3.14 - theta;
    int T = (atan2((xtarget - x),(ytarget - y)) - tmpTheta)*25/3.15;
    int velocity = 255 - abs(T);

    this->leftJoint->SetVelocity(0, (velocity + T)/this->wheelRadius);
    this->rightJoint->SetVelocity(0, (velocity - T)/this->wheelRadius);
  }


}