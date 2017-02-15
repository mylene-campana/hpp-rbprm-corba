// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include "hpp/corbaserver/rbprm/rbprmbuilder.hh"
#include "rbprmbuilder.impl.hh"
#include "hpp/rbprm/rbprm-device.hh"
#include "hpp/rbprm/rbprm-validation.hh"
#include "hpp/rbprm/interpolation/rbprm-path-interpolation.hh"
#include "hpp/rbprm/interpolation/limb-rrt.hh"
#include "hpp/rbprm/interpolation/com-rrt.hh"
#include "hpp/rbprm/interpolation/com-trajectory.hh"
#include "hpp/rbprm/interpolation/spline/effector-rrt.hh"
#include "hpp/rbprm/stability/stability.hh"
#include "hpp/rbprm/sampling/sample-db.hh"
#include "hpp/model/urdf/util.hh"
#include <Eigen/Geometry>

#include "hpp/core/straight-path.hh"
#include "hpp/model/joint.hh"
#include "hpp/core/config-validations.hh"
#include "hpp/core/collision-validation-report.hh"
#include <hpp/core/subchain-path.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/collision-validation.hh>
#include <fstream>
#include <hpp/rbprm/planner/dynamic-planner.hh>
//#include <hpp/rbprm/planner/parabola-planner.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif

#include <hpp/rbprm/fullbodyBallistic/parabola-library.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-path.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-interpolation.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-planner.hh>
#include "hpp/rbprm/planner/parabola-path.hh"
#include <hpp/rbprm/fullbodyBallistic/timed-ballistic-path.hh>
//#include <polytope/stability_margin.h>

namespace hpp {
  namespace rbprm {
    namespace impl {
      using model::displayConfig;

      hpp::floatSeq* vectorToFloatseq (const hpp::core::vector_t& input)
      {
	CORBA::ULong size = (CORBA::ULong) input.size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [(CORBA::ULong)i] = input [i];
	}
	return q_ptr;
      }

    RbprmBuilder::RbprmBuilder ()
    : POA_hpp::corbaserver::rbprm::RbprmBuilder()
    , romLoaded_(false)
    , fullBodyLoaded_(false)
    , bindShooter_()
    , analysisFactory_(0)
    , bindAnalysis_()
    , fillGenerateContactState_ (false)
    {
        // NOTHING
    }

    void RbprmBuilder::loadRobotRomModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        try
        {
            hpp::model::DevicePtr_t romDevice = model::Device::create (robotName);
            romDevices_.insert(std::make_pair(robotName, romDevice));
            hpp::model::urdf::loadRobotModel (romDevice,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        romLoaded_ = true;
    }

    void RbprmBuilder::loadRobotCompleteModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        if(!romLoaded_)
        {
            std::string err("Rom must be loaded before loading complete model") ;
            hppDout (error, err );
            throw hpp::Error(err.c_str());
        }
        try
        {
            hpp::model::RbPrmDevicePtr_t device = hpp::model::RbPrmDevice::create (robotName, romDevices_);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            // Add device to the planner
            problemSolver_->robot (device);
            problemSolver_->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::loadFullBodyRobot(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        try
        {
            model::DevicePtr_t device = model::Device::create (robotName);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            fullBody_ = rbprm::RbPrmFullBody::create(device);
            problemSolver_->pathValidationType ("Discretized",0.05); // reset to avoid conflict with rbprm path
            problemSolver_->robot (fullBody_->device_);
            problemSolver_->resetProblem();
            problemSolver_->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        fullBodyLoaded_ = true;
        analysisFactory_ = new sampling::AnalysisFactory(fullBody_);
    }

    void RbprmBuilder::loadFullBodyRobotFromExistingRobot() throw (hpp::Error)
    {
        try
        {
            fullBody_ = rbprm::RbPrmFullBody::create(problemSolver_->problem()->robot());
            problemSolver_->pathValidationType ("Discretized",0.05); // reset to avoid conflict with rbprm path
            problemSolver_->robot (fullBody_->device_);
            problemSolver_->resetProblem();
            problemSolver_->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        fullBodyLoaded_ = true;
        analysisFactory_ = new sampling::AnalysisFactory(fullBody_);
    }

    hpp::floatSeq* RbprmBuilder::getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        Eigen::VectorXd config = fullBody_->device_->currentConfiguration ();
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        config.segment(sample.startRank_, sample.length_) = sample.configuration_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(config.rows()));
        for(std::size_t i=0; i< _CORBA_ULong(config.rows()); i++)
          (*dofArray)[(_CORBA_ULong)i] = config [i];
        return dofArray;
    }


    hpp::floatSeq* RbprmBuilder::getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        const fcl::Vec3f& position = sample.effectorPosition_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(3));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        return dofArray;
    }


    typedef Eigen::Matrix <value_type, 4, 3, Eigen::RowMajor> Matrix43;
    typedef Eigen::Matrix <value_type, 4, 3, Eigen::RowMajor> Rotation;
    typedef Eigen::Ref<Matrix43> Ref_matrix43;

    std::vector<fcl::Vec3f> computeRectangleContact(const rbprm::RbPrmFullBodyPtr_t device,
                                                    const rbprm::State& state)
    {
        device->device_->currentConfiguration(state.configuration_);
        device->device_->computeForwardKinematics();
        std::vector<fcl::Vec3f> res;
        const rbprm::T_Limb& limbs = device->GetLimbs();
        rbprm::RbPrmLimbPtr_t limb;
        Matrix43 p; Eigen::Matrix3d R;
        for(std::map<std::string, fcl::Vec3f>::const_iterator cit = state.contactPositions_.begin();
            cit != state.contactPositions_.end(); ++cit)
        {
            const std::string& name = cit->first;
            const fcl::Vec3f& position = cit->second;
            limb = limbs.at(name);
            const fcl::Vec3f& normal = state.contactNormals_.at(name);
            const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
            const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
            const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
            const fcl::Vec3f offset = rotation * limb->offset_;
            const double& lx = limb->x_, ly = limb->y_;
            p << lx,  ly, 0,
                 lx, -ly, 0,
                -lx, -ly, 0,
                -lx,  ly, 0;
            if(limb->contactType_ == _3_DOF)
            {
                //create rotation matrix from normal
                fcl::Vec3f z_fcl = state.contactNormals_.at(name);
                Eigen::Vector3d z,x,y;
                for(int i =0; i<3; ++i) z[i] = z_fcl[i];
                x = z.cross(Eigen::Vector3d(0,-1,0));
                if(x.norm() < 10e-6)
                {
                    y = z.cross(fcl::Vec3f(1,0,0));
                    y.normalize();
                    x = y.cross(z);
                }
                else
                {
                    x.normalize();
                    y = z.cross(x);
                }
                R.block<3,1>(0,0) = x;
                R.block<3,1>(0,1) = y;
                R.block<3,1>(0,2) = z;
                /*for(std::size_t i =0; i<4; ++i)
                {
                    res.push_back(position + (R*(p.row(i).transpose())) + offset);
                    res.push_back(state.contactNormals_.at(name));
                }*/
                res.push_back(position + (R*(offset)));
                res.push_back(state.contactNormals_.at(name));
            }
            else
            {
                const fcl::Matrix3f& fclRotation = state.contactRotation_.at(name);
                for(int i =0; i< 3; ++i)
                    for(int j =0; j<3;++j)
                        R(i,j) = fclRotation(i,j);
                fcl::Vec3f z_axis(0,0,1);
                fcl::Matrix3f rotationLocal = tools::GetRotationMatrix(z_axis, limb->normal_);
                for(std::size_t i =0; i<4; ++i)
                {
                    res.push_back(position + (R*(rotationLocal*(p.row(i).transpose() + limb->offset_))));
                    res.push_back(state.contactNormals_.at(name));
                }
            }
        }
        return res;
    }

    std::vector<fcl::Vec3f> computeRectangleContact(const rbprm::RbPrmFullBodyPtr_t device,
                                                    const rbprm::State& state,
                                                    const std::string& limbName)
    {
        device->device_->currentConfiguration(state.configuration_);
        device->device_->computeForwardKinematics();
        std::vector<fcl::Vec3f> res;
        const rbprm::T_Limb& limbs = device->GetLimbs();
        rbprm::RbPrmLimbPtr_t limb;
        Matrix43 p; Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); Eigen::Matrix3d cFrame = Eigen::Matrix3d::Identity();
        for(std::map<std::string, fcl::Vec3f>::const_iterator cit = state.contactPositions_.begin();
            cit != state.contactPositions_.end(); ++cit)
        {
            const std::string& name = cit->first;
            if(limbName == name)
            {
                const fcl::Vec3f& position = cit->second;
                limb = limbs.at(name);
                const fcl::Vec3f& normal = state.contactNormals_.at(name);
                const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
                const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
                const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
                const fcl::Vec3f offset = rotation * limb->offset_;
                const double& lx = limb->x_, ly = limb->y_;
                p << lx,  ly, 0,
                     lx, -ly, 0,
                    -lx, -ly, 0,
                    -lx,  ly, 0;
                if(limb->contactType_ == _3_DOF)
                {
                    //create rotation matrix from normal
                    Eigen::Vector3d z,x,y;
                    for(int i =0; i<3; ++i) z[i] = normal[i];
                    x = z.cross(Eigen::Vector3d(0,-1,0));
                    if(x.norm() < 10e-6)
                    {
                        y = z.cross(fcl::Vec3f(1,0,0));
                        y.normalize();
                        x = y.cross(z);
                    }
                    else
                    {
                        x.normalize();
                        y = z.cross(x);
                    }
                    cFrame.block<3,1>(0,0) = x;
                    cFrame.block<3,1>(0,1) = y;
                    cFrame.block<3,1>(0,2) = z;
                }
                fcl::Transform3f roWorld, roEffector;
                roWorld.setRotation(state.contactRotation_.at(name));
                roWorld.setTranslation(position);
                roEffector = roWorld; roEffector.inverse();
                fcl::Vec3f z_axis(0,0,1);
                fcl::Matrix3f rotationLocal = tools::GetRotationMatrix(z_axis, limb->normal_);
                if(limb->contactType_ == _3_DOF)
                {
                    fcl::Vec3f pworld = position +  offset;
                    res.push_back((roEffector * pworld).getTranslation());
                    res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                }
                else
                {
                    for(std::size_t i =0; i<4; ++i)
                    {
                        /*if(limb->contactType_ == _3_DOF)
                        {
                            fcl::Vec3f pworld = position + (cFrame*(p.row(i).transpose())) + offset;
                            res.push_back((roEffector * pworld).getTranslation());
                            res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                        }
                        else*/
                        {
                            res.push_back(rotationLocal*(p.row(i).transpose()) + limb->offset_);
                            res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                        }
                    }
                }
                return res;
            }
        }
        return res;
    }

    model::Configuration_t dofArrayToConfig (const std::size_t& deviceDim,
      const hpp::floatSeq& dofArray)
    {
        std::size_t configDim = (std::size_t)dofArray.length();
        // Fill dof vector with dof array.
        model::Configuration_t config; config.resize (configDim);
        for (std::size_t iDof = 0; iDof < configDim; iDof++) {
            config [iDof] = (double)dofArray[(_CORBA_ULong)iDof];
        }
        // fill the vector by zero
        hppDout (info, "config dimension: " <<configDim
           <<",  deviceDim "<<deviceDim);
        if(configDim != deviceDim){
            throw hpp::Error ("dofVector Does not match");
        }
        return config;
    }

    model::Configuration_t dofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeq& dofArray)
    {
        return dofArrayToConfig(robot->configSize(), dofArray);
    }

    T_Configuration doubleDofArrayToConfig (const std::size_t& deviceDim,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        std::size_t configsDim = (std::size_t)doubleDofArray.length();
        T_Configuration res;
        for (_CORBA_ULong iConfig = 0; iConfig < configsDim; iConfig++)
        {
            res.push_back(dofArrayToConfig(deviceDim, doubleDofArray[iConfig]));
        }
        return res;
    }

    T_Configuration doubleDofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        return doubleDofArrayToConfig(robot->configSize(), doubleDofArray);
    }

    hpp::floatSeqSeq* RbprmBuilder::getEffectorPosition(const char* lb, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try
        {
            const std::string limbName(lb);
            const RbPrmLimbPtr_t limb = fullBody_->GetLimbs().at(limbName);
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
            fullBody_->device_->currentConfiguration(config);
            fullBody_->device_->computeForwardKinematics();
            State state;
            state.configuration_ = config;
	    hppDout (info, "HARDCODE LIMB CONTACT STATUS");
            state.contacts_[limbName] = true;
            const fcl::Vec3f position = limb->effector_->currentTransformation().getTranslation();
            state.contactPositions_[limbName] = position;
            state.contactNormals_[limbName] = fcl::Vec3f(0,0,1);
            state.contactRotation_[limbName] = limb->effector_->currentTransformation().getRotation();
            std::vector<fcl::Vec3f> poss = (computeRectangleContact(fullBody_, state));

            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();
            res->length ((_CORBA_ULong)poss.size ());
            for(std::size_t i = 0; i < poss.size(); ++i)
            {
                _CORBA_ULong size = (_CORBA_ULong) (3);
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for(std::size_t j=0; j<3; j++)
                {
                    dofArray[j] = poss[i][j];
                }
                (*res) [(_CORBA_ULong)i] = floats;
            }
            return res;
        }
        catch (const std::exception& exc)
        {
            throw hpp::Error (exc.what ());
        }
    }


    CORBA::UShort RbprmBuilder::getNumSamples(const char* limb) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        return (CORBA::UShort)(lit->second->sampleContainer_.samples_.size());
    }

    floatSeq *RbprmBuilder::getOctreeNodeIds(const char* limb) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const sampling::T_VoxelSampleId& ids =  lit->second->sampleContainer_.samplesInVoxels_;
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length((_CORBA_ULong)ids.size());
        sampling::T_VoxelSampleId::const_iterator it = ids.begin();
        for(std::size_t i=0; i< _CORBA_ULong(ids.size()); ++i, ++it)
        {
          (*dofArray)[(_CORBA_ULong)i] = (double)it->first;
        }
        return dofArray;
    }

    double RbprmBuilder::getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const sampling::SampleDB& database = lit->second->sampleContainer_;
        if (database.samples_.size() <= sampleId)
        {
            std::string err("unexisting sample id " + sampleId);
            throw Error (err.c_str());
        }
        sampling::T_Values::const_iterator cit = database.values_.find(std::string(valueName));
        if(cit == database.values_.end())
        {
            std::string err("value not existing in database " + std::string(valueName));
            throw Error (err.c_str());
        }
        return cit->second[sampleId];
    }

    double RbprmBuilder::getEffectorDistance(unsigned short state1, unsigned short state2) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            return rbprm::effectorDistance(lastStatesComputed_[s1], lastStatesComputed_[s2]);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    std::vector<std::string> stringConversion(const hpp::Names_t& dofArray)
    {
        std::vector<std::string> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (_CORBA_ULong iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(std::string(dofArray[iDof]));
        }
        return res;
    }

    std::vector<double> doubleConversion(const hpp::floatSeq& dofArray)
    {
        std::vector<double> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (_CORBA_ULong iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(dofArray[iDof]);
        }
        return res;
    }


    void RbprmBuilder::setFilter(const hpp::Names_t& roms) throw (hpp::Error)
    {
        bindShooter_.romFilter_ = stringConversion(roms);
    }


    void RbprmBuilder::setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error)
    {
        std::string name (romName);
				std::vector<std::string> affNames = stringConversion(affordances);
        bindShooter_.affFilter_.erase(name);
        bindShooter_.affFilter_.insert(std::make_pair(name,affNames));
    }

    void RbprmBuilder::boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error)
    {
        std::vector<double> limits = doubleConversion(limitszyx);
        if(limits.size() !=6)
        {
            throw Error ("Can not bound SO3, array of 6 double required");
        }
        bindShooter_.so3Bounds_ = limits;

    }

    double RbprmBuilder::projectStateToCOMEigen(unsigned short stateId, const model::Configuration_t& com_target) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            }
            State s = lastStatesComputed_[stateId];
            bool succes (false);
            hpp::model::Configuration_t c = rbprm::interpolation::projectOnCom(fullBody_, problemSolver_->problem(),s,com_target,succes);
            if(succes)
            {
                lastStatesComputed_[stateId].configuration_ = c;
                lastStatesComputedTime_[stateId].second.configuration_ = c;
                return 1.;
            }
            return 0;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

      CORBA::Short RbprmBuilder::createState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs, const bool isInContact, const double time) throw (hpp::Error)
    {
      // if isInContact is True, all given limbs are supposed to be in contact
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        State state;
        state.configuration_ = config;
        std::vector<std::string> names = stringConversion(contactLimbs);
	for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end(); ++cit)
	  {
	    rbprm::RbPrmLimbPtr_t limb = fullBody_->GetLimbs().at(*cit);
	    const std::string& limbName = *cit;
	    hppDout (info, "HARDCODE LIMB CONTACT STATUS with isInContact");
	    state.contacts_[limbName] = isInContact;
	    if (isInContact) {
	      const fcl::Vec3f position = limb->effector_->currentTransformation().getTranslation();
	      state.contactPositions_[limbName] = position;
	      state.contactNormals_[limbName] = limb->effector_->currentTransformation().getRotation() * limb->normal_;
	      state.contactRotation_[limbName] = limb->effector_->currentTransformation().getRotation();
	    }
	  }
        lastStatesComputed_.push_back(state);
	if (time != -1) {
	  lastStatesComputedTime_.push_back(std::make_pair(time,state));
	}
        return lastStatesComputed_.size()-1;
    }

    double RbprmBuilder::projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com) throw (hpp::Error)
    {        
        model::Configuration_t com_target = dofArrayToConfig (3, com);
        return projectStateToCOMEigen(stateId, com_target);
    }

      hpp::floatSeq* RbprmBuilder::generateContacts(const hpp::floatSeq& configuration, const hpp::floatSeq& direction, const bool noStability, const bool useFlexionPose) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[(_CORBA_ULong)i];
            }
	    fullBody_->noStability_ = noStability;
	    const affMap_t &affMap = problemSolver_->map
	      <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
	    if (affMap.empty ()) {
	      throw hpp::Error ("No affordances found. Unable to generate Contacts.");
			}
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
	    rbprm::State state = rbprm::ComputeContacts(fullBody_,config,
							affMap,
							bindShooter_.affFilter_,
							dir);

           core::Configuration_t q;
           if(useFlexionPose && flexionPose_.size() > 0) {
	     hppDout (info, "generate contacts using flexionPose");
             q = rbprm::computeContactPose(state,flexionPose_,fullBody_);
	   }
           else 
             q = state.configuration_;
	    std::queue<std::string> contactStack = state.contactOrder_;
	    const std::size_t contactNumber = contactStack.size ();
	    fcl::Vec3f normalAv = (0,0,0);
	    while(!contactStack.empty())
	      {
		const std::string name = contactStack.front();
		contactStack.pop();
		const fcl::Vec3f& normal = state.contactNormals_.at(name);
		for (std::size_t j = 0; j < 3; j++) {
		  normalAv [j] += normal [j]/contactNumber;
		}
	      }
	    normalAv.normalize ();
	    hppDout (info, "normed normalAv= " << normalAv);
	    // If robot has ECS, fill new average-normal in it
	    if (fullBody_->device_->extraConfigSpace ().dimension () > 3) {
	      const std::size_t indexECS = fullBody_->device_->configSize () - fullBody_->device_->extraConfigSpace ().dimension ();
	      for (std::size_t i = 0; i < 3; i++)
		q [indexECS + i] = normalAv [i];
	    }
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(_CORBA_ULong(q.rows()));
            for(std::size_t i=0; i< _CORBA_ULong(q.rows()); i++)
              (*dofArray)[(_CORBA_ULong)i] = q [i];

	    // display contacting limbs for debug
	    for(rbprm::T_Limb::const_iterator lit = fullBody_->GetLimbs().begin();lit != fullBody_->GetLimbs().end(); ++lit){
	      if(state.contacts_[lit->first]){ // limb is in contact
		hppDout(info,"contacting limbs in generateContacts: " << lit->first);
	      } else
		hppDout(info,"NOT contacting limbs in generateContacts: " << lit->first);
	    }
	    if (fillGenerateContactState_) { // set by fillGenerateContactState
	      hppDout(info,"generateContactState is filled");
	      generateContactState_ = state;
	    }

            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::generateGroundContact(const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f z(0,0,1);
            ValidationReportPtr_t report = ValidationReportPtr_t(new CollisionValidationReport);
            core::BasicConfigurationShooterPtr_t  shooter = core::BasicConfigurationShooter::create(fullBody_->device_);
            for(int i =0; i< 1000; ++i)
            {
                core::DevicePtr_t device = fullBody_->device_->clone();
                std::vector<std::string> names = stringConversion(contactLimbs);
                core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(device,"proj", 1e-4, 40);
                //hpp::tools::LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
                for(std::vector<std::string>::const_iterator cit = names.begin(); cit !=names.end(); ++cit)
                {
                    rbprm::RbPrmLimbPtr_t limb = fullBody_->GetLimbs().at(*cit);
                    fcl::Transform3f localFrame, globalFrame;
                    localFrame.setTranslation(-limb->offset_);
                    std::vector<bool> posConstraints;
                    std::vector<bool> rotConstraints;
                    posConstraints.push_back(false);posConstraints.push_back(false);posConstraints.push_back(true);
                    rotConstraints.push_back(true);rotConstraints.push_back(true);rotConstraints.push_back(true);
                    proj->add(core::NumericalConstraint::create (constraints::Position::create("",fullBody_->device_,
                                                                                               limb->effector_,
                                                                                               globalFrame,
                                                                                               localFrame,
                                                                                               posConstraints)));
                    if(limb->contactType_ == hpp::rbprm::_6_DOF)
                    {
                        // rotation matrix around z
                        value_type theta = 2*(value_type(rand()) / value_type(RAND_MAX) - 0.5) * M_PI;
                        fcl::Matrix3f r = tools::GetZRotMatrix(theta);
                        fcl::Matrix3f rotation = r * limb->effector_->initialPosition ().getRotation();
                        proj->add(core::NumericalConstraint::create (constraints::Orientation::create("",fullBody_->device_,
                                                                                                      limb->effector_,
                                                                                                      fcl::Transform3f(rotation),
                                                                                                      rotConstraints)));
                    }
                }
                ConfigurationPtr_t configptr = shooter->shoot();
                Configuration_t config = *configptr;
                if(proj->apply(config) && config[2]> 0.3)
                {
                    if(problemSolver_->problem()->configValidations()->validate(config,report))
                    {
                        State tmp;
                        for(std::vector<std::string>::const_iterator cit = names.begin(); cit !=names.end(); ++cit)
                        {
                            std::string limbId = *cit;
                            rbprm::RbPrmLimbPtr_t limb = fullBody_->GetLimbs().at(*cit);
			    hppDout (info, "HARDCODE LIMB CONTACT STATUS");
                            tmp.contacts_[limbId] = true;
                            tmp.contactPositions_[limbId] = limb->effector_->currentTransformation().getTranslation();
                            tmp.contactRotation_[limbId] = limb->effector_->currentTransformation().getRotation();
                            tmp.contactNormals_[limbId] = z;
                            tmp.configuration_ = config;
                            ++tmp.nbContacts;
                        }
                        if(stability::IsStable(fullBody_,tmp)>=0)
                        {
                            config[0]=0;
                            config[1]=0;
                            hpp::floatSeq* dofArray = new hpp::floatSeq();
                            dofArray->length(_CORBA_ULong(config.rows()));
                            for(std::size_t j=0; j< config.rows(); j++)
                            {
                              (*dofArray)[(_CORBA_ULong)j] = config [j];
                            }
                            return dofArray;
                        }
                    }
                }
            }
            throw (std::runtime_error("could not generate contact configuration after 1000 trials"));
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::getContactSamplesIds(const char* limbname,
                                        const hpp::floatSeq& configuration,
                                        const hpp::floatSeq& direction) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[(_CORBA_ULong)i];
            }
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
            model::Configuration_t save = fullBody_->device_->currentConfiguration();
            fullBody_->device_->currentConfiguration(config);

            sampling::T_OctreeReport finalSet;
            rbprm::T_Limb::const_iterator lit = fullBody_->GetLimbs().find(std::string(limbname));
            if(lit == fullBody_->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + std::string(limbname) + " to robot; limb not defined.");
            }
            const RbPrmLimbPtr_t& limb = lit->second;
            fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration
                        // TODO fix as in rbprm-fullbody.cc!!
            std::vector<sampling::T_OctreeReport> reports(problemSolver_->collisionObstacles().size());
            std::size_t i (0);
            //#pragma omp parallel for
            for(model::ObjectVector_t::const_iterator oit = problemSolver_->collisionObstacles().begin();
                oit != problemSolver_->collisionObstacles().end(); ++oit, ++i)
            {
                sampling::GetCandidates(limb->sampleContainer_, transform, *oit, dir, reports[i]);
            }
            for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
                cit != reports.end(); ++cit)
            {
                finalSet.insert(cit->begin(), cit->end());
            }
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length((_CORBA_ULong)finalSet.size());
            sampling::T_OctreeReport::const_iterator candCit = finalSet.begin();
            for(std::size_t i=0; i< _CORBA_ULong(finalSet.size()); ++i, ++candCit)
            {
              (*dofArray)[(_CORBA_ULong)i] = (double)candCit->sample_->id_;
            }
            fullBody_->device_->currentConfiguration(save);
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::getSamplesIdsInOctreeNode(const char* limb,
                                                           double octreeNodeId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            long ocId ((long)octreeNodeId);
            const T_Limb& limbs = fullBody_->GetLimbs();
            T_Limb::const_iterator lit = limbs.find(std::string(limb));
            if(lit == limbs.end())
            {
                std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
                throw Error (err.c_str());
            }
            const sampling::T_VoxelSampleId& sampleIds =  lit->second->sampleContainer_.samplesInVoxels_;
            sampling::T_VoxelSampleId::const_iterator cit = sampleIds.find(ocId);
            if(cit == sampleIds.end())
            {
                std::stringstream ss; ss << ocId;
                std::string err("No octree node with id " + ss.str() + "was defined for robot" + fullBody_->device_->name());
                throw Error (err.c_str());
            }
            const sampling::VoxelSampleId& ids = cit->second;
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length((_CORBA_ULong)ids.second);
            std::size_t sampleId = ids.first;
            for(std::size_t i=0; i< _CORBA_ULong(ids.second); ++i, ++sampleId)
            {
              (*dofArray)[(_CORBA_ULong)i] = (double)sampleId;
            }
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                               unsigned short samples, const char* heuristicName, double resolution, const char *contactType, double disableEffectorCollision) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f off, norm;
            for(std::size_t i =0; i <3; ++i)
            {
                off[i] = offset[(_CORBA_ULong)i];
                norm[i] = normal[(_CORBA_ULong)i];
            }
	    hppDout (info, "norm= " << norm);
            ContactType cType = hpp::rbprm::_6_DOF;
            if(std::string(contactType) == "_3_DOF")
            {
                cType = hpp::rbprm::_3_DOF;
            }
            fullBody_->AddLimb(std::string(id), std::string(limb), std::string(effector), off, norm, x, y,
                               problemSolver_->collisionObstacles(), samples,heuristicName,resolution,cType,disableEffectorCollision > 0.5);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    void RbprmBuilder::addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues, double disableEffectorCollision) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            std::string fileName(databasePath);
            fullBody_->AddLimb(fileName, std::string(id), problemSolver_->collisionObstacles(), heuristicName, loadValues > 0.5,
                               disableEffectorCollision > 0.5);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void SetPositionAndNormal(rbprm::State& state,
			      hpp::rbprm::RbPrmFullBodyPtr_t fullBody,
			      const hpp::floatSeq& configuration,
			      std::vector<std::string>& names)
    {
        core::Configuration_t old = fullBody->device_->currentConfiguration();
        model::Configuration_t config = dofArrayToConfig (fullBody->device_, configuration);
        fullBody->device_->currentConfiguration(config);
        fullBody->device_->computeForwardKinematics();

        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            rbprm::T_Limb::const_iterator lit = fullBody->GetLimbs().find(*cit);
            if(lit == fullBody->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + (*cit) + " to robot; limb not defined");
            }
	    const core::JointPtr_t joint = fullBody->device_->getJointByName(lit->second->effector_->name());
	    const fcl::Transform3f& transform =  joint->currentTransformation ();
	    const fcl::Matrix3f& rot = transform.getRotation();
	    state.contactPositions_[*cit] = transform.getTranslation();
	    state.contactRotation_[*cit] = rot;
	    const fcl::Vec3f z = transform.getRotation()*lit->second->normal_;
	    state.contactNormals_[*cit] = z;
	    state.contacts_[*cit] = true;
	    state.contactOrder_.push(*cit);
        }        
        state.nbContacts = state.contactNormals_.size() ;
        state.configuration_ = config;
        state.robustness =  stability::IsStable(fullBody,state);
        state.stable = state.robustness >= 0;
        fullBody->device_->currentConfiguration(old);
    }

      void RbprmBuilder::setStartState (const hpp::floatSeq& configuration,
					const hpp::Names_t& contactLimbs)
	throw (hpp::Error)
    {
        try
        {
	  if (!fillGenerateContactState_) {
	    // create state from given configuration, assuming ALL limbs are in contact ...
	    hppDout (info, "(startState) SetPositionAndNormal to set ALL contacts");
            std::vector<std::string> names = stringConversion(contactLimbs);
            SetPositionAndNormal(startState_,fullBody_, configuration, names);
	  } else {
	    // use state from generateContact (contact data already filled)
	    hppDout (info, "use generateContactState_ for startState_");
	    startState_ = generateContactState_;
	  }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    hpp::floatSeq* RbprmBuilder::computeContactForConfig(const hpp::floatSeq& configuration, const char *limbNam) throw (hpp::Error)
    {
        State state;
        std::string limb(limbNam);
        try
        {
            std::vector<std::string> limbs; limbs.push_back(limbNam);
            SetPositionAndNormal(state,fullBody_, configuration, limbs);

            const std::vector<fcl::Vec3f>& positions = computeRectangleContact(fullBody_,state,limb);
            _CORBA_ULong size = (_CORBA_ULong) (positions.size () * 3);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(size);
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    (*dofArray)[(_CORBA_ULong)j] = positions[h][k];
                }
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }

    }

    void RbprmBuilder::setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
	  if (!fillGenerateContactState_) {
	    // create state from given configuration, assuming ALL limbs are in contact ...
	    hppDout (info, "(endState) SetPositionAndNormal to set ALL contacts");
            std::vector<std::string> names = stringConversion(contactLimbs);
            SetPositionAndNormal(endState_,fullBody_, configuration, names);
	    } else {
	    // use state from generateContact (contact data already filled)
	    hppDout (info, "use generateContactState_ for endState_");
	    endState_ = generateContactState_;
	  }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

      /*namespace{
  // similar to TimeStatesToStates   Mylene test if useless
      void copyStateFrames(const rbprm::T_StateFrame& stateFrames, rbprm::T_State& states )
      {
        states.clear();
        for(rbprm::T_StateFrame::const_iterator cit = stateFrames.begin(); cit != stateFrames.end(); ++cit)
        {
          states.push_back(cit->second);
        }
      }
      }*/

    std::vector<State> TimeStatesToStates(const T_StateFrame& ref)
    {
        std::vector<State> res;
        for(CIT_StateFrame cit = ref.begin(); cit != ref.end(); ++cit)
        {
            res.push_back(cit->second);
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error)
    {
        try
        {
            if(startState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("Start state not initialized, can not interpolate ");
            }
            if(endState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("End state not initialized, can not interpolate ");
            }
            T_Configuration configurations = doubleDofArrayToConfig(fullBody_->device_, configs);
            const affMap_t &affMap = problemSolver_->map
                        <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
            if (affMap.empty ())
            {
                throw hpp::Error ("No affordances found. Unable to interpolate.");
            }
            hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = rbprm::interpolation::RbPrmInterpolation::create(fullBody_,startState_,endState_);
            lastStatesComputedTime_ = interpolator->Interpolate(affMap, bindShooter_.affFilter_,configurations,robustnessTreshold, filterStates != 0);
            lastStatesComputed_ = TimeStatesToStates(lastStatesComputedTime_);
            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();

            res->length ((_CORBA_ULong)lastStatesComputed_.size ());
            std::size_t i=0;
            std::size_t id = 0;
            for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin(); cit != lastStatesComputed_.end(); ++cit, ++id)
            {
                std::cout << "ID " << id;
                cit->print();
                const core::Configuration_t config = cit->configuration_;
                _CORBA_ULong size = (_CORBA_ULong) config.size ();
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for (model::size_type j=0 ; j < config.size() ; ++j) {
                  dofArray[j] = config [j];
                }
                (*res) [(_CORBA_ULong)i] = floats;
                ++i;
            }
            return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeqSeq* contactCone(RbPrmFullBodyPtr_t& fullBody, State& state, const double friction)
    {
        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        std::pair<stability::MatrixXX, stability::VectorX> cone = stability::ComputeCentroidalCone(fullBody, state, friction);
        res->length ((_CORBA_ULong)cone.first.rows());
        _CORBA_ULong size = (_CORBA_ULong) cone.first.cols()+1;
        for(int i=0; i < cone.first.rows(); ++i)
        {
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the row in dofseq
            for (int j=0 ; j < cone.first.cols() ; ++j)
            {
                dofArray[j] = cone.first(i,j);
            }
            dofArray[size-1] =cone.second[i];
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    hpp::floatSeqSeq* RbprmBuilder::getContactCone(unsigned short stateId, double friction) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            }
            return contactCone(fullBody_, lastStatesComputed_[stateId],friction);
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    State intermediary(const State& firstState, const State& thirdState, unsigned short& cId, bool& success)
    {
        success = false;
        std::vector<std::string> breaks;
        thirdState.contactBreaks(firstState, breaks);
        if(breaks.size() > 1)
        {
            throw std::runtime_error ("too many contact breaks between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }
        if(breaks.size() == 1)
        {
            State intermediary(firstState);
            intermediary.RemoveContact(*breaks.begin());
            success = true;
            return intermediary;
        }
        return firstState;
    }

    hpp::floatSeqSeq* RbprmBuilder::getContactIntermediateCone(unsigned short stateId, double friction) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId+1)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId+1)));
            }
            bool success;
            State intermediaryState = intermediary(lastStatesComputed_[stateId], lastStatesComputed_[stateId+1],stateId,success);
            if(!success)
            {
                throw std::runtime_error ("No contact breaks, hence no intermediate state from state " + std::string(""+(stateId)));
            }
            return contactCone(fullBody_,intermediaryState, friction);
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    core::PathPtr_t makePath(DevicePtr_t device,
                             const ProblemPtr_t& problem,
                             model::ConfigurationIn_t q1,
                             model::ConfigurationIn_t q2)
    {
        // TODO DT
        return problem->steeringMethod()->operator ()(q1,q2);
    }

    model::Configuration_t addRotation(CIT_Configuration& pit, const model::value_type& u,
                              model::ConfigurationIn_t q1, model::ConfigurationIn_t q2,
                              model::ConfigurationIn_t ref)
    {
        model::Configuration_t res = ref;
        res.head(3) = *pit;
        res.segment<4>(3) = tools::interpolate(q1, q2, u);
        return res;
    }

    core::PathVectorPtr_t addRotations(const T_Configuration& positions,
                                       model::ConfigurationIn_t q1, model::ConfigurationIn_t q2,
                                       model::ConfigurationIn_t ref, DevicePtr_t device,
                                       const ProblemPtr_t& problem)
    {
        core::PathVectorPtr_t res = core::PathVector::create(device->configSize(), device->numberDof());
        model::value_type size_step = 1 /(model::value_type)(positions.size());
        model::value_type u = 0.;
        CIT_Configuration pit = positions.begin();
        model::Configuration_t previous = addRotation(pit, 0., q1, q2, ref), current;
        ++pit;
        for(;pit != positions.end()-1; ++pit, u+=size_step)
        {
            current = addRotation(pit, u, q1, q2, ref);
            res->appendPath(makePath(device,problem, previous, current));
            previous = current;
        }
        // last configuration is exactly q2
        current = addRotation(pit, 1., q1, q2, ref);
        res->appendPath(makePath(device,problem, previous, current));
        return res;
    }

   core::PathVectorPtr_t generateTrunkPath(RbPrmFullBodyPtr_t& fullBody, core::ProblemSolverPtr_t problemSolver, const hpp::floatSeqSeq& rootPositions,
                          const model::Configuration_t q1, const model::Configuration_t q2) throw (hpp::Error)
    {
        try
        {
            T_Configuration positions = doubleDofArrayToConfig(3, rootPositions);
            if(positions.size() <2)
            {
                throw std::runtime_error("generateTrunkPath requires at least 2 configurations to generate path");
            }
            return addRotations(positions,q1,q2,fullBody->device_->currentConfiguration(),
                                                 fullBody->device_,problemSolver->problem());
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    CORBA::Short RbprmBuilder::generateRootPath(const hpp::floatSeqSeq& rootPositions,
                          const hpp::floatSeq& q1Seq, const hpp::floatSeq& q2Seq) throw (hpp::Error)
    {
        try
        {
            model::Configuration_t q1 = dofArrayToConfig(4, q1Seq), q2 = dofArrayToConfig(4, q2Seq);
            return problemSolver_->addPath(generateTrunkPath(fullBody_, problemSolver_, rootPositions, q1, q2));
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                          const hpp::floatSeqSeq& accelerations,
                                          const double dt) throw (hpp::Error)
     {
         try
         {
             T_Configuration c = doubleDofArrayToConfig(3, positions);
             T_Configuration cd = doubleDofArrayToConfig(3, velocities);
             T_Configuration cdd = doubleDofArrayToConfig(3, accelerations);
             if(c.size() <2)
             {
                 throw std::runtime_error("generateComTraj requires at least 2 configurations to generate path");
             }
             core::PathVectorPtr_t res = core::PathVector::create(3, 3);
             if(cdd.size() != c.size()-1 || cdd.size() != cd.size())
             {
                 std::cout << c.size() << " " << cd.size() << " " << cdd.size() << std::endl;
                 throw std::runtime_error("in generateComTraj, positions and accelerations vector should have the same size");
             }
             CIT_Configuration cit = c.begin(); ++cit;
             CIT_Configuration cdit = cd.begin();
             CIT_Configuration cddit = cdd.begin();
             int i = 0;
             for(;cit != c.end(); ++cit, ++cdit, ++cddit, ++i)
             {
                 res->appendPath(interpolation::ComTrajectory::create(*(cit-1),*cit,*cdit,*cddit,dt));
             }
             return problemSolver_->addPath(res);
         }
         catch(std::runtime_error& e)
         {
             throw Error(e.what());
         }
     }


    floatSeqSeq* RbprmBuilder::computeContactPoints(unsigned short cId) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + 1)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId+1)));
        }
        const State& firstState = lastStatesComputed_[cId], thirdState = lastStatesComputed_[cId+1];
        std::vector<std::vector<fcl::Vec3f> > allStates;
        allStates.push_back(computeRectangleContact(fullBody_, firstState));
        std::vector<std::string> creations;
        bool success(false);
        State intermediaryState = intermediary(firstState, thirdState, cId, success);
        if(success)
        {
            allStates.push_back(computeRectangleContact(fullBody_, intermediaryState));
        }
        thirdState.contactCreations(firstState, creations);
        if(creations.size() == 1)
        {
            allStates.push_back(computeRectangleContact(fullBody_, thirdState));
        }
        if(creations.size() > 1)
        {
            throw std::runtime_error ("too many contact creations between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::computeContactPointsForLimb(unsigned short cId, const char *limbName) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + 1)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId+1)));
        }
        std::string limb(limbName);
        const State& firstState = lastStatesComputed_[cId], thirdState = lastStatesComputed_[cId+1];
        std::vector<std::vector<fcl::Vec3f> > allStates;
        allStates.push_back(computeRectangleContact(fullBody_, firstState, limb));
        std::vector<std::string> creations;
        bool success(false);
        State intermediaryState = intermediary(firstState, thirdState, cId, success);
        if(success)
        {
            allStates.push_back(computeRectangleContact(fullBody_, intermediaryState, limb));
        }
        thirdState.contactCreations(firstState, creations);
        if(creations.size() == 1)
        {
            allStates.push_back(computeRectangleContact(fullBody_, thirdState, limb));
        }
        if(creations.size() > 1)
        {
            throw std::runtime_error ("too many contact creations between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::interpolate(double timestep, double path, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error)
    {
        try
        {
        unsigned int pathId = int(path);
        if(startState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("Start state not initialized, cannot interpolate ");
        }
        if(endState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("End state not initialized, cannot interpolate ");
        }

        if(problemSolver_->paths().size() <= pathId)
        {
            throw std::runtime_error ("No path computed, cannot interpolate ");
        }
        const affMap_t &affMap = problemSolver_->map
					<std::vector<boost::shared_ptr<model::CollisionObject> > > ();
        if (affMap.empty ())
        {
            throw hpp::Error ("No affordances found. Unable to interpolate.");
        }

        hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = 
					rbprm::interpolation::RbPrmInterpolation::create(fullBody_,startState_,endState_,problemSolver_->paths()[pathId]);
        lastStatesComputedTime_ = interpolator->Interpolate(affMap, bindShooter_.affFilter_,
                    timestep,robustnessTreshold, filterStates != 0);
		lastStatesComputed_ = TimeStatesToStates(lastStatesComputedTime_);

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        res->length ((_CORBA_ULong)lastStatesComputed_.size ());
        std::size_t i=0;
        std::size_t id = 0;
        for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin();
					cit != lastStatesComputed_.end(); ++cit, ++id)
        {
            /*std::cout << "ID " << id;
            cit->print();*/
            const core::Configuration_t config = cit->configuration_;
            _CORBA_ULong size = (_CORBA_ULong) config.size ();
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < config.size() ; ++j) {
              dofArray[j] = config [j];
            }
            (*res) [(_CORBA_ULong)i] = floats;
            ++i;
        }
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short AddPath(core::PathPtr_t path, core::ProblemSolverPtr_t ps)
    {
        core::PathVectorPtr_t resPath = core::PathVector::create(path->outputSize(), path->outputDerivativeSize());
        resPath->appendPath(path);
        return ps->addPath(resPath);
    }

    CORBA::Short RbprmBuilder::limbRRT(double state1, double state2, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            //create helper
//            /interpolation::TimeConstraintHelper helper(fullBody_, problemSolver_->problem());
            core::PathPtr_t path = interpolation::limbRRT(fullBody_,problemSolver_->problem(),
                                                                          lastStatesComputed_.begin()+s1,lastStatesComputed_.begin()+s2, numOptimizations);
            return AddPath(path,problemSolver_);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::limbRRTFromRootPath(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            unsigned int pathId = (unsigned int)(path);
            if(problemSolver_->paths().size() <= pathId)
            {
                throw std::runtime_error ("No path computed, cannot interpolate ");
            }
            core::PathPtr_t path = interpolation::limbRRTFromPath(fullBody_,problemSolver_->problem(), problemSolver_->paths()[pathId],
                                                                          lastStatesComputedTime_.begin()+s1,lastStatesComputedTime_.begin()+s2, numOptimizations);
            return AddPath(path,problemSolver_);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::configToPath(const hpp::floatSeqSeq& configs) throw (hpp::Error)
    {
        T_Configuration positions =  doubleDofArrayToConfig(fullBody_->device_, configs);
        core::PathVectorPtr_t res = core::PathVector::create(fullBody_->device_->configSize(),
                                                             fullBody_->device_->numberDof());
        //for(CIT_Configuration pit = positions.begin();pit != positions.end()-1; ++pit)
        for(CIT_Configuration pit = positions.begin();pit != positions.end()-200; ++pit)
        {
            res->appendPath(makePath(fullBody_->device_,problemSolver_->problem(), *pit,*(pit+1)));
        }
        return problemSolver_->addPath(res);
    }

    CORBA::Short RbprmBuilder::comRRT(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
	    hppDout (info, "s1= " << s1 << ", s2= " << s2);
// temp
//assert(s2 == s1 +1);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
	      hppDout (info, "lastStatesComputed_.size ()= " << lastStatesComputed_.size ());
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            unsigned int pathId = (unsigned int)(path);
            if(problemSolver_->paths().size() <= pathId)
            {
                throw std::runtime_error ("No path computed, cannot interpolate ");
            }
            core::PathPtr_t path = interpolation::comRRT(fullBody_,problemSolver_->problem(), problemSolver_->paths()[pathId],
                                                                          *(lastStatesComputed_.begin()+s1),*(lastStatesComputed_.begin()+s2), numOptimizations);
            return AddPath(path,problemSolver_);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    core::Configuration_t project_or_throw(rbprm::RbPrmFullBodyPtr_t fulllBody, ProblemPtr_t problem, const State& state, const fcl::Vec3f& targetCom)
    {
        bool success(false);
        core::Configuration_t res = rbprm::interpolation::projectOnCom(fulllBody, problem,state,targetCom, success);
        if(!success)
        {
            throw std::runtime_error("could not project state on COM constraint");
        }
        return res;
    }

    hpp::floatSeq* RbprmBuilder::rrt(t_rrt functor,  double state1,
                                    unsigned short cT1, unsigned short cT2, unsigned short cT3,
                                    unsigned short numOptimizations)  throw (hpp::Error)
    {
        try
        {
            std::vector<CORBA::Short> pathsIds;
            std::size_t s1((std::size_t)state1), s2((std::size_t)state1+1);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            const core::PathVectors_t& paths = problemSolver_->paths();
            if(paths.size() -1 < std::max(cT1, std::max(cT2, cT3)))
            {
                throw std::runtime_error("in comRRTFromPos, at least one com trajectory is not present in problem solver");
            }
            State& state1=lastStatesComputed_[s1], state2=lastStatesComputed_[s2];

            State s1Bis(state1);
            s1Bis.configuration_ = project_or_throw(fullBody_, problemSolver_->problem(),s1Bis,paths[cT1]->end().head<3>());

            State s2Bis(state2);
            s2Bis.configuration_ = project_or_throw(fullBody_, problemSolver_->problem(),s2Bis,paths[cT2]->end().head<3>());

            core::PathVectorPtr_t resPath = core::PathVector::create(fullBody_->device_->configSize(), fullBody_->device_->numberDof());



            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            fullBody_->device_->currentConfiguration(s1Bis.configuration_);
            if(!(problemSolver_->problem()->configValidations()->validate(s1Bis.configuration_, rport)
                    && problemSolver_->problem()->configValidations()->validate(s2Bis.configuration_, rport)))
            {
                std::cout << "could not project without collision at state " << s1  << std::endl;
//throw std::runtime_error ("could not project without collision at state " + s1 );
            }

            {
                core::PathPtr_t p1 = interpolation::comRRT(fullBody_,problemSolver_->problem(), paths[cT1],
                        state1,s1Bis, numOptimizations,true);
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p1->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p1,intervals);
                resPath->appendPath(reducedPath);
                pathsIds.push_back(AddPath(p1,problemSolver_));
            }


            {
                core::PathPtr_t p2 =(*functor)(fullBody_,problemSolver_->problem(), paths[cT2],
                    s1Bis,s2Bis, numOptimizations,true);
                pathsIds.push_back(AddPath(p2,problemSolver_));
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p2->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p2,intervals);
                resPath->appendPath(reducedPath);
            }

            //if(s2Bis.configuration_ != state2.configuration_)
            {
                core::PathPtr_t p3= interpolation::comRRT(fullBody_,problemSolver_->problem(), paths[cT3],
                        s2Bis,state2, numOptimizations,true);
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p3->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p3,intervals);
                resPath->appendPath(reducedPath);
                pathsIds.push_back(AddPath(p3,problemSolver_));
            }
            pathsIds.push_back(AddPath(resPath,problemSolver_));

            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(pathsIds.size());
            for(std::size_t i=0; i< pathsIds.size(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = pathsIds[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::comRRTFromPos(double state1,
                                               unsigned short cT1,
                                               unsigned short cT2,
                                               unsigned short cT3,
                                               unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::comRRT, state1, cT1, cT2, cT3, numOptimizations);

    }

    hpp::floatSeq* RbprmBuilder::effectorRRT(double state1,
                                             unsigned short cT1,
                                             unsigned short cT2,
                                             unsigned short cT3,
                                             unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::effectorRRT, state1, cT1, cT2, cT3, numOptimizations);
    }

    hpp::floatSeq* RbprmBuilder::effectorRRTFromPath(double state1,
                                                    unsigned short refpath, double path_from, double path_to,
                                                    unsigned short cT1,
                                                    unsigned short cT2,
                                                    unsigned short cT3,
                                                    unsigned short numOptimizations,
                                                    const Names_t &trackedEffector) throw (hpp::Error)
    {
        try
        {
            std::vector<CORBA::Short> pathsIds;
            std::size_t s1((std::size_t)state1), s2((std::size_t)state1+1);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            const core::PathVectors_t& paths = problemSolver_->paths();
            if(paths.size() -1 < std::max(cT1, std::max(cT2, cT3)))
            {
                throw std::runtime_error("in comRRTFromPos, at least one com trajectory is not present in problem solver");
            }
            const State& state1=lastStatesComputed_[s1], state2=lastStatesComputed_[s2];

            core::PathVectorPtr_t comPath = core::PathVector::create(3,3);
            comPath->appendPath(paths[cT1]);
            comPath->appendPath(paths[cT2]);
            comPath->appendPath(paths[cT3]);
            std::vector<std::string> trackedEffectorNames = stringConversion(trackedEffector);
            core::PathPtr_t refFullBody = problemSolver_->paths()[refpath]->extract(std::make_pair(path_from, path_to));
            core::PathPtr_t p2 =interpolation::effectorRRTFromPath(fullBody_,problemSolver_->problem(), comPath,
                    state1,state2, numOptimizations,true, refFullBody, trackedEffectorNames);
            pathsIds.push_back(AddPath(p2,problemSolver_));

            // reduce path to remove extradof
            core::SizeInterval_t interval(0, p2->initial().rows()-1);
            core::SizeIntervals_t intervals;
            intervals.push_back(interval);
            PathPtr_t reducedPath = core::SubchainPath::create(p2,intervals);
            core::PathVectorPtr_t resPath = core::PathVector::create(fullBody_->device_->configSize(), fullBody_->device_->numberDof());
            resPath->appendPath(reducedPath);

            pathsIds.push_back(AddPath(resPath,problemSolver_));
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(pathsIds.size());
            for(std::size_t i=0; i< pathsIds.size(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = pathsIds[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::projectToCom(double state, const hpp::floatSeq& targetCom) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size () < state)
            {
                throw std::runtime_error ("did not find a states at indicated index: " + std::string(""+(std::size_t)(state)));
            }
            model::Configuration_t config = dofArrayToConfig (std::size_t(3), targetCom);
            fcl::Vec3f comTarget; for(int i =0; i<3; ++i) comTarget[i] = config[i];
            model::Configuration_t  res = project_or_throw(fullBody_,problemSolver_->problem(), lastStatesComputed_[state],comTarget);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(res.rows());
            for(std::size_t i=0; i< res.rows(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = res[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getConfigAtState(unsigned short state) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size () < state)
            {
                throw std::runtime_error ("did not find a state at indicated index: " + std::string(""+(std::size_t)(state)));
            }
            model::Configuration_t res = lastStatesComputed_[state].configuration_;
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(res.rows());
            for(std::size_t i=0; i< res.rows(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = res[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::saveComputedStates(const char* outfilename) throw (hpp::Error)
    {
        std::stringstream ss;
        ss << lastStatesComputed_.size()-2 << "\n";
        std::vector<rbprm::State>::iterator cit = lastStatesComputed_.begin()+1;
        int i = 0;
        ss << i++ << " ";
        cit->print(ss);
        for(std::vector<rbprm::State>::iterator cit2 = lastStatesComputed_.begin()+2;
            cit2 != lastStatesComputed_.end()-1; ++cit2, ++cit, ++i)
        {
            cit2->robustness = stability::IsStable(this->fullBody_, *cit2);
            ss << i<< " ";
            cit2->print(ss,*cit);
        }
        std::ofstream outfile;
        outfile.open(outfilename);
        if (outfile.is_open())
        {
            outfile << ss.rdbuf();
            outfile.close();
        }
        else
        {
            std::string error("Cannot open outfile " + std::string(outfilename));
            throw Error(error.c_str());
        }
    }

    void RbprmBuilder::saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error)
    {
        try
        {
        std::string limbName(limbname);
        std::ofstream fout;
        fout.open(filepath, std::fstream::out | std::fstream::app);
        rbprm::saveLimbInfoAndDatabase(fullBody_->GetLimbs().at(limbName),fout);
        //sampling::saveLimbDatabase(fullBody_->GetLimbs().at(limbName)->sampleContainer_,fout);
        fout.close();
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeqSeq* RbprmBuilder::getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try
        {
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        model::Configuration_t save = fullBody_->device_->currentConfiguration();
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        const std::map<std::size_t, fcl::CollisionObject*>& boxes =
                fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.boxes_;
        const double resolution = fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.resolution_;
        std::size_t i =0;
        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();
        res->length ((_CORBA_ULong)boxes.size ());
        for(std::map<std::size_t, fcl::CollisionObject*>::const_iterator cit = boxes.begin();
            cit != boxes.end();++cit,++i)
        {
            fcl::Vec3f position = (*cit->second).getTranslation();
            _CORBA_ULong size = (_CORBA_ULong) 4;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < 3 ; ++j) {
                dofArray[j] = position[j];
            }
            dofArray[3] = resolution;
            (*res) [(_CORBA_ULong)i] = floats;
        }
        fullBody_->device_->currentConfiguration(save);
        fullBody_->device_->computeForwardKinematics();
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getOctreeBox(const char* limbName, double octreeNodeId) throw (hpp::Error)
    {
        try
        {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        long ocId ((long)octreeNodeId);
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limbName));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limbName) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const std::map<std::size_t, fcl::CollisionObject*>& boxes =
                fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.boxes_;
        std::map<std::size_t, fcl::CollisionObject*>::const_iterator cit = boxes.find(ocId);
        if(cit == boxes.end())
        {
            std::stringstream ss; ss << ocId;
            std::string err("No octree node with id " + ss.str() + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const fcl::CollisionObject* box = cit->second;
        const fcl::Vec3f& pos = box->getTransform().getTranslation();
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length(4);
        for(std::size_t i=0; i< 3; ++i)
        {
          (*dofArray)[(_CORBA_ULong)i] = pos[i];
        }
        (*dofArray)[(_CORBA_ULong)3] = fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.resolution_;
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    CORBA::Short RbprmBuilder::isLimbInContact(const char* limbName, double state1) throw (hpp::Error)
    {
        try
        {
            std::size_t s((std::size_t)state1);
            if(lastStatesComputed_.size () < s)
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s));
            }
            const std::map<std::string, fcl::Vec3f> & contacts = lastStatesComputed_[s].contactPositions_;
            if(contacts.find(std::string(limbName))!= contacts.end())
                return 1;
            return 0;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::isLimbInContactIntermediary(const char* limbName, double state1) throw (hpp::Error)
    {
        try
        {
            std::size_t s((std::size_t)state1);
            if(lastStatesComputed_.size () < s+1)
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s));
            }
            const State& state1 = lastStatesComputed_[s];
            const State& state2 = lastStatesComputed_[s+1];
            bool unused;
            short unsigned cId = s;
            const State state = intermediary(state1, state2,cId,unused);
            const std::map<std::string, fcl::Vec3f> & contacts = state.contactPositions_;
            if(contacts.find(std::string(limbName))!= contacts.end())
                return 1;
            return 0;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try{
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        model::Configuration_t save = fullBody_->device_->currentConfiguration();
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody_->GetLimbs().at(std::string(limbName));
        const fcl::Transform3f transform = limb->octreeRoot();
        const fcl::Quaternion3f& quat = transform.getQuatRotation();
        const fcl::Vec3f& position = transform.getTranslation();
        hpp::floatSeq *dofArray;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(7));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        for(std::size_t i=0; i< 4; i++)
          (*dofArray)[(_CORBA_ULong)i+3] = quat [i];
        fullBody_->device_->currentConfiguration(save);
        fullBody_->device_->computeForwardKinematics();
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::isConfigBalanced(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error)
    {
        try{
        rbprm::State testedState;
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        model::Configuration_t save = fullBody_->device_->currentConfiguration();
        std::vector<std::string> names = stringConversion(contactLimbs);
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody_->GetLimbs().at(std::string(*cit));
	    hppDout (info, "HARDCODE LIMB CONTACT STATUS");
            testedState.contacts_[*cit] = true;
            testedState.contactPositions_[*cit] = limb->effector_->currentTransformation().getTranslation();
            testedState.contactRotation_[*cit] = limb->effector_->currentTransformation().getRotation();
            // normal given by effector normal
            const fcl::Vec3f normal = limb->effector_->currentTransformation().getRotation() * limb->normal_;
            testedState.contactNormals_[*cit] = normal;
            testedState.configuration_ = config;
            ++testedState.nbContacts;
        }
        fullBody_->device_->currentConfiguration(save);
        fullBody_->device_->computeForwardKinematics();
        if (stability::IsStable(fullBody_, testedState) >= robustnessTreshold)
        {
            return (CORBA::Short)(1);
        }
        else
        {
            return (CORBA::Short)(0);
        }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    void RbprmBuilder::runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error)
    {
        try
        {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        std::string eval(analysis);
        if (eval == "all")
        {
            for(sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.begin();
                analysisit != analysisFactory_->evaluate_.end(); ++ analysisit)
            {
                for(T_Limb::const_iterator cit = fullBody_->GetLimbs().begin(); cit !=fullBody_->GetLimbs().end();++cit)
                {
                    sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (cit->second->sampleContainer_);
                    sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
                }
            }
        }
        else
        {
            sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.find(std::string(eval));
            if(analysisit == analysisFactory_->evaluate_.end())
            {
                std::string err("No analysis named  " + eval + "was defined for analyzing database sample");
                throw Error (err.c_str());
            }
            for(T_Limb::const_iterator cit = fullBody_->GetLimbs().begin(); cit !=fullBody_->GetLimbs().end();++cit)
            {
                sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (cit->second->sampleContainer_);
                sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
            }
        }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error)
    {
        try
        {
        rbprm::sampling::ValueBound bounds;
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        T_Limb::const_iterator lit = fullBody_->GetLimbs().find(std::string(limbname));
        if(lit == fullBody_->GetLimbs().end())
        {
            std::string err("No limb " + std::string(limbname) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        std::string eval(analysis);
        if (eval == "all")
        {
            for(sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.begin();
                analysisit != analysisFactory_->evaluate_.end(); ++ analysisit)
            {
                sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (lit->second->sampleContainer_);
                sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
                bounds = sampleDB.valueBounds_[analysisit->first];
            }
        }
        else
        {
            sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.find(std::string(eval));
            if(analysisit == analysisFactory_->evaluate_.end())
            {
                std::string err("No analysis named  " + eval + "was defined for analyzing database sample");
                throw Error (err.c_str());
            }
            sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (lit->second->sampleContainer_);
            sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
            bounds = sampleDB.valueBounds_[analysisit->first];
        }
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length(2);
        (*dofArray)[0] = bounds.first;
        (*dofArray)[1] = bounds.second;
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::dumpProfile(const char* logFile) throw (hpp::Error)
    {
        try
        {
            #ifdef PROFILE
                RbPrmProfiler& watch = getRbPrmProfiler();
                std::ofstream fout;
                fout.open(logFile, std::fstream::out | std::fstream::app);
                std::ostream* fp = &fout;
                watch.report_all_and_count(2,*fp);
                fout.close();
            #else
                throw(std::runtime_error("PROFILE PREPOC variable undefined, cannot dump profile"));
            #endif
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver)
    {
      hppDout (info, "set problem solver");
        problemSolver_ = problemSolver;
        bindShooter_.problemSolver_ = problemSolver;
        //bindHeuristics_.problemSolver_ = problemSolver;
        //bind shooter creator to hide problem as a parameter and respect signature

        // add rbprmshooter
        problemSolver->add<core::ConfigurationShooterBuilder_t>("RbprmShooter",
                                                   boost::bind(&BindShooter::create, boost::ref(bindShooter_), _1));
        problemSolver->add<core::PathValidationBuilder_t>("RbprmPathValidation",
                                                   boost::bind(&BindShooter::createPathValidation, boost::ref(bindShooter_), _1, _2));
        problemSolver->add<core::PathPlannerBuilder_t>("DynamicPlanner",DynamicPlanner::createWithRoadmap);
        problemSolver->add<core::PathPlannerBuilder_t>("BallisticPlanner",BallisticPlanner::createWithRoadmap);
        problemSolver->add <core::SteeringMethodBuilder_t> ("RBPRMKinodynamic", SteeringMethodKinodynamic::create);


    }

      // --------------------------------------------------------------------
  
      void RbprmBuilder::setRbShooter () throw (hpp::Error)
      {
	try {
	  core::Configuration_t q_proj;
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  rbShooter_ = bindShooter_.create (robot);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------
    
      hpp::floatSeq* RbprmBuilder::rbShoot () throw (hpp::Error)
      {
	try {
	  if (!rbShooter_)
	    throw std::runtime_error ("rb shooter was not set");
	  core::Configuration_t q_proj;
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  q_proj = *(rbShooter_->shoot ());
	  hppDout (info, "q_proj: " << displayConfig (q_proj));

	  hpp::floatSeq *dofArray_out = 0x0;
	  dofArray_out = new hpp::floatSeq();
	  dofArray_out->length (robot->configSize ());
	  for(std::size_t i=0; i<robot->configSize (); i++)
	    (*dofArray_out)[i] = q_proj (i);
	  return dofArray_out;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------
      
      hpp::floatSeq* RbprmBuilder::setOrientation(const hpp::floatSeq& dofArray)
        throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (),
						     dofArray);
	q = rbprm::setOrientation (robot, q);
	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = q (i);
	return dofArray_out;
      }
	  
      // --------------------------------------------------------------------
	  
      void RbprmBuilder::isRbprmValid (const hpp::floatSeq& dofArray,
				       CORBA::Boolean& trunkValidity,
				       CORBA::Boolean& romValidity,
				       CORBA::String_out report)
	throw (hpp::Error) {
	try {
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  model::RbPrmDevicePtr_t rbRobot = boost::static_pointer_cast<model::RbPrmDevice>(robot);
	  core::ValidationReportPtr_t valReport, unusedreport, valReport2;
	  std::vector<std::string>& filter = bindShooter_.romFilter_;
	  rbprm::RbPrmValidationPtr_t validator (hpp::rbprm::RbPrmValidation::create(rbRobot, filter, bindShooter_.affFilter_, bindShooter_.affMap_, core::ObjectVector_t(), bindShooter_.nbFilterMatch_));

	  core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (),
						      dofArray);
	  trunkValidity = validator->trunkValidation_->validate(q, valReport);
	  romValidity = validator->validateRoms(q, filter);
	  report = CORBA::string_dup ("");

	  for(int i = 0; i < filter.size (); i++) {
	    hppDout (info, "bindShooter_.romFilter_= " << bindShooter_.romFilter_ [i]);
	    hppDout (info, "filter= " << filter [i]);
	  }

	  if(!trunkValidity || !romValidity) {
	    hppDout (info, "config is not RB-valid");
	    hppDout (info, "trunkValidity= " << trunkValidity);
	    hppDout (info, "romValidity= " << romValidity);
	    std::ostringstream oss;
	    oss << *valReport;
	    report = CORBA::string_dup(oss.str ().c_str ());
	    oss << *valReport2;
	    report = CORBA::string_dup(oss.str ().c_str ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::interpolateBallisticPath (const CORBA::UShort pathId,
						   const double u_offset,
						   const CORBA::Boolean timed)
	throw (hpp::Error){
        try {
	  std::size_t pid = (std::size_t) pathId;
	  if(startState_.configuration_.rows() == 0)
	      throw std::runtime_error ("Start state not initialized, can not interpolate ");
	  if(endState_.configuration_.rows() == 0)
	      throw std::runtime_error ("End state not initialized, can not interpolate ");

	  if(problemSolver_->paths().size() <= pid)
	      throw std::runtime_error ("No path computed, cannot interpolate ");

	  const core::PathVectorConstPtr_t path = problemSolver_->paths()[pid];

	  const core::PathPtr_t subpath1 = (*path).pathAtRank (0);
	  const ParabolaPathPtr_t pp1 = 
	    boost::dynamic_pointer_cast<ParabolaPath>(subpath1);

	  const std::size_t subPathNumber = path->numberPaths ();
	  hpp::rbprm::BallisticInterpolationPtr_t interpolator = 
	    rbprm::BallisticInterpolation::create((problemSolver_->problem ()),
						  fullBody_, startState_,
						  endState_, path);
	  const core::PathPtr_t subpath = (*path).pathAtRank (0);
	  const ParabolaPathPtr_t pp = 
	    boost::dynamic_pointer_cast<ParabolaPath>(subpath);
	  rbprm::T_StateFrame stateFrames;

	  if (extendingPose_.rows() > 0)
	    interpolator->extendingPose (extendingPose_);
	  else
	    hppDout (info, "no extending pose was provided to interpolator");
	  if (flexionPose_.rows() > 0)
	    interpolator->flexionPose (flexionPose_);
	  else
	    hppDout (info, "no flexion pose was provided to interpolator");
	  if (takeoffContactPose_.rows() > 0)
	    interpolator->takeoffContactPose (takeoffContactPose_);
	  else
	    hppDout (info, "no takeoff-contact pose was provided to interpolator");
	  if (landingContactPose_.rows() > 0)
	    interpolator->landingContactPose (landingContactPose_);
	  else
	    hppDout (info, "no landing-contact pose was provided to interpolator");
	  const affMap_t &affMap = problemSolver_->map
	    <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
	  if (!affMap.empty ()) {
	    interpolator->affordanceFilters (bindShooter_.affFilter_);
	    interpolator->affordanceMap (affMap);
	  }
	  else
	    hppDout (error, "No affordances can be given to interpolator");

	  core::PathVectorPtr_t result = core::PathVector::create (fullBody_->device_->configSize (), fullBody_->device_->numberDof ());
	  rbprm::T_PathVectorBP interpResult;
	  // each path-vector corresponds to an interpolated parabola

	  if (subPathNumber == 1)
	    interpResult = interpolator->InterpolateDirectPath(u_offset, &stateFrames);
	  else
	    interpResult = interpolator->InterpolateFullPath(u_offset, &stateFrames);

	  // update the stacks of states (e.g. for comRRT)
	  lastStatesComputedTime_.clear ();
	  lastStatesComputedTime_.resize (stateFrames.size());
	  lastStatesComputed_.clear ();
	  lastStatesComputed_.resize (stateFrames.size());
	  for (std::size_t i = 0; i < stateFrames.size(); i++) {
	    lastStatesComputedTime_ [i] = stateFrames [i];
	    lastStatesComputed_ [i] = stateFrames [i].second;
	  }

	  hppDout (info, "interpResult.size ()= " << interpResult.size ());

	  // concatenate the interpolation results, according to
	  // time-parametrization or not.
	  if (!timed) {
	    hppDout (info, "space-parametrized interpolation");
	    for (std::size_t i = 0; i < interpResult.size (); i++) {
	      // no time parametrization, simply concatenate path-vectors
	      result->appendPath (interpResult [i].first);
	    }
	  } else {
	    // time parametrization, convert and concatenate
	    hppDout (info, "time-parametrized interpolation");
	    for (std::size_t i = 0; i < interpResult.size (); i++) {
	      result->appendPath (rbprm::TimedBallisticPath::create (interpResult [i]));
	    } // for each i_th parabola in interpResult
	  }
	  
	  problemSolver_->addPath (result);
	}
	catch(std::runtime_error& e)
	  {
	    throw Error(e.what());
	  }
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* RbprmBuilder::generateWaypointContacts
      (CORBA::UShort pathId) throw (hpp::Error){
        try {
	  std::size_t pid = (std::size_t) pathId;
	  if(problemSolver_->paths().size() <= pid) {
	    throw std::runtime_error ("No path computed");
	  }

	  core::DevicePtr_t robot = problemSolver_->robot ();
	  const CORBA::ULong configSize = robot->configSize(); // with ECS
	  hppDout (info, "robot configSize: " << configSize);
	  const CORBA::ULong ecsSize = robot->extraConfigSpace ().dimension ();
	  const core::PathVectorPtr_t path = problemSolver_->paths () [pid];
	  const std::size_t num_subpaths  = (*path).numberPaths ();
	  std::vector<core::Configuration_t> configs;
	  const affMap_t affordances = bindShooter_.affMap_;
	  const std::map<std::string, std::vector<std::string> > affFilters = bindShooter_.affFilter_;
	  fullBody_->noStability_ = true; // disable stability for waypoints
	  
	  for (std::size_t i = 1; i < num_subpaths; i++) {
	    core::PathPtr_t subpath = (*path).pathAtRank (i); // trunk-size!!
	    const std::size_t trunkSize = subpath->initial ().size ();
	    hppDout (info, "trunkSize: " << trunkSize);
	    model::Configuration_t q1 (configSize);
	    hppDout (info, "waypoint before fill: " << displayConfig(subpath->initial ()));
	    for (std::size_t j = 0; j < configSize - ecsSize; j++) {
	      if (j < trunkSize)
		q1 [j] = subpath->initial () [j];
	      else
		q1 [j] = 0;
	    }
	    for (std::size_t k = 0; k < ecsSize; k++)
	      q1 [configSize - ecsSize + k] = subpath->initial () [trunkSize - ecsSize + k];
	    hppDout (info, "waypoint before contacts: " << displayConfig(q1));
	    fcl::Vec3f dir;
	    dir [0] = 0; dir [1] = 0; dir [2] = 1;
	    rbprm::State state =
	      rbprm::ComputeContacts(fullBody_, q1, affordances,
				     affFilters, dir);
	    model::Configuration_t q = state.configuration_;
	    hppDout (info, "waypoint after contacts: " << displayConfig(q));
	    configs.push_back (q);
	  }
	  hppDout (info, "configs.size = " << configs.size ());
	  
	  hpp::floatSeq* dofArray;
	  dofArray = new hpp::floatSeq ();
	  dofArray->length (configSize);
	  hpp::floatSeqSeq *configSequence;
	  configSequence = new hpp::floatSeqSeq ();
	  configSequence->length ((CORBA::ULong) configs.size ());
	  for (std::size_t i = 0; i < configs.size (); i++) {
	    dofArray = vectorToFloatseq (configs [i]);
	    (*configSequence) [(CORBA::ULong) i] = *dofArray;
	  }
	  return configSequence;
	}
	catch(std::runtime_error& e)
	  {
	    throw Error(e.what());
	  }
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* RbprmBuilder::fillConfiguration
      (const hpp::floatSeq& dofArray, const CORBA::UShort fullSize)
	throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	std::size_t trunkSize = q.size ();
	std::size_t ecsSize = robot->extraConfigSpace ().dimension ();
	core::Configuration_t result (fullSize);
	hppDout (info, "original config= " << displayConfig (q));
	for (std::size_t j = 0; j < fullSize - ecsSize; j++) {
	  if (j < trunkSize)
	    result [j] = q [j];
	  else
	    result [j] = 0;
	}
	// copy extra-configs at the end of the config
	hppDout (info, "ecsSize= " << ecsSize);
	for (std::size_t k = 0; k < ecsSize; k++)
	  result [fullSize - ecsSize + k] = q [trunkSize - ecsSize + k];
	hppDout (info, "filled config= " << displayConfig (result));
	
	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = result (i);
	return dofArray_out;
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::setNumberFilterMatch
      (const CORBA::UShort nbFilterMatch) throw (hpp::Error) {
	bindShooter_.nbFilterMatch_ = nbFilterMatch;
	problemSolver_->problem ()->configValidations ()->selectFirst ()->setSizeParameter (nbFilterMatch);
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* RbprmBuilder::getsubPathsV0Vimp (const char* Vquery,
							 CORBA::UShort pathId)
	throw (hpp::Error) {
	std::size_t pid = (std::size_t) pathId;
	if(problemSolver_->paths().size() <= pid) {
	  throw std::runtime_error ("No path computed");
	}
	std::string Vquery_str = std::string(Vquery);
	if(Vquery_str.compare ("V0") != 0 && Vquery_str.compare ("Vimp") != 0) {
	  throw std::runtime_error ("Query problem, ask for V0 or Vimp");
	}
	const core::PathVectorPtr_t path = problemSolver_->paths () [pid];
	const std::size_t num_subpaths  = (*path).numberPaths ();
	core::vector_t V (3);
	std::vector<core::vector_t> V_vector;
	for (std::size_t i = 0; i < num_subpaths; i++) {
	  const core::PathPtr_t subpath = (*path).pathAtRank (i);
	  ParabolaPathPtr_t pp = 
	    boost::dynamic_pointer_cast<ParabolaPath>(subpath);
	  if (Vquery_str.compare("V0") == 0)
	    V = pp->V0_;
	  else
	    V = pp->Vimp_;
	  V_vector.push_back (V);
	}
	hpp::floatSeq* vArray;
	vArray = new hpp::floatSeq ();
	vArray->length (3);
	hpp::floatSeqSeq *vSequence;
	vSequence = new hpp::floatSeqSeq ();
	vSequence->length ((CORBA::ULong) V_vector.size ());
	for (std::size_t i = 0; i < V_vector.size (); i++) {
	  for (std::size_t k = 0; k < 3; k++) {
	    (*vArray) [(CORBA::ULong) k] = V_vector [i][k];
	  }
	  (*vSequence) [(CORBA::ULong) i] = *vArray;
	}
	return vSequence;
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::rotateAlongPath (const CORBA::UShort pathId,
                      const bool fullbody, const bool rotateAfterJump, const bool trunkOrientation, const bool getCloseToContact)
	throw (hpp::Error) {
	std::size_t pid = (std::size_t) pathId;
	if(problemSolver_->paths().size() <= pid) {
	  throw std::runtime_error ("No path computed");
	}
	try {
	  const core::PathVectorPtr_t path = problemSolver_->paths () [pid];
	  core::PathPtr_t tmpPath;
	  const std::size_t num_subpaths  = (*path).numberPaths ();
	  std::vector <core::Configuration_t> waypoints;
	  core::Configuration_t config, q1, q2, q1rot, q2rot;
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  const size_type index = robot->configSize ()
	    - robot->extraConfigSpace ().dimension ();
	  core::PathVectorPtr_t newPath =
	    core::PathVector::create (robot->configSize (),
				      robot->numberDof ());
	  bool success;
	  core::Configuration_t qTmp;
	  core::ValidationReportPtr_t report;

	  for (std::size_t i = 0; i < num_subpaths; i++) { // store waypoints
	    tmpPath = (*path).pathAtRank (i);
	    config = (*tmpPath) (0, success);
	    waypoints.push_back (config);
	    hppDout (info, "wp(i): " << displayConfig (waypoints [i]));
	  }
	  // last config
	  tmpPath = (*path).pathAtRank (num_subpaths - 1);
	  waypoints.push_back ((*tmpPath) (tmpPath->length (), success));

	  // update theta values
	  value_type theta_i;
	  value_type alpha_i;
    if(rotateAfterJump){
      theta_i = atan2 (waypoints [1][1]-waypoints [0][1],
					waypoints [1][0]-waypoints [0][0]);
	    waypoints [0][index + 3] = theta_i;
    }
	  for (std::size_t i = 0 + rotateAfterJump; i < (waypoints.size () - 1 + rotateAfterJump); i++) {
	    // theta_(i,i+1)
	    theta_i = atan2 (waypoints [i+1 - rotateAfterJump][1]-waypoints [i - rotateAfterJump][1],
					waypoints [i+1- rotateAfterJump][0]-waypoints [i- rotateAfterJump][0]);
	    //hppDout (info, "theta_i: " << theta_i);
	    waypoints [i][index + 3] = theta_i;


	  }
	  // ! last orientation (qEnd) = last theta_i
    if(rotateAfterJump)
    {
        waypoints [waypoints.size () - 1][index + 3] = atan2 (waypoints [waypoints.size()-1][1]-waypoints [waypoints.size()-2][1],
        waypoints [waypoints.size()-1][0]-waypoints [waypoints.size()-2][0]);
    }
    else  
      waypoints [waypoints.size () - 1][index + 3] = theta_i;

	  // update waypoints orientations
	  //const JointPtr_t skullJoint = robot_->getJointByName ("Skull");
	  //const std::size_t rank = skullJoint->rankInConfiguration ();
	  // TODO: if fullbody, use OrientationConstraint for skullJoint ??
	  for (std::size_t i = 0; i < waypoints.size (); i++) {
	    qTmp = rbprm::setOrientation (robot, waypoints [i]);
	    if (problemSolver_->problem ()->configValidations()->validate(qTmp,report)) {
	      hppDout (info, "waypoint with alpha orientation is valid");
	      waypoints[i] = qTmp;
	      } else {
	      hppDout (info, "waypoint with alpha orientation is NOT valid= " << displayConfig(qTmp));
	      }
	    //waypoints [i][rank] = skullJoint->lowerBound (0);
	    hppDout (info, "new wp(i): " << displayConfig (waypoints [i]));
	  }


      // Test Pierre : (set orientation of Z trunk axis to the direction of alpha0)
    if(trunkOrientation) {
      Eigen::Vector3d yTheta;
      Eigen::Quaterniond qr ,qi,qf;
      
      for(std::size_t i = 0 ; i< waypoints.size() -1 ; i++ ){
        qTmp = waypoints[i];
        alpha_i = (boost::dynamic_pointer_cast<ParabolaPath>((*path).pathAtRank (i)))->coefficients()[4];
        theta_i = qTmp[index+3];
        yTheta = Eigen::Vector3d(-sin(theta_i), cos(theta_i),0);
        qr= Eigen::AngleAxisd((M_PI/2)-alpha_i, yTheta); // rotation needed
        qi = Eigen::Quaterniond(qTmp[3],qTmp[4],qTmp[5],qTmp[6]);
        qf = qr*qi;
        
        qTmp[3]= qf.w();
        qTmp[4]= qf.x();
        qTmp[5]= qf.y();
        qTmp[6]= qf.z();
        if (problemSolver_->problem ()->configValidations()->validate(qTmp,report)) {
          hppDout (info, "waypoint with alpha orientation is valid");
          waypoints[i] = qTmp;
        } else {
          hppDout (info, "waypoint with alpha orientation is NOT valid= " << displayConfig(qTmp));
        }
      }
      // goal state : use last alpha and theta value
      qTmp = waypoints[waypoints.size () - 1];
      yTheta = Eigen::Vector3d(-sin(theta_i), cos(theta_i),0);
      qr= Eigen::AngleAxisd((M_PI/2)-alpha_i, yTheta); // rotation needed
      qi = Eigen::Quaterniond(qTmp[3],qTmp[4],qTmp[5],qTmp[6]);
      qf = qr*qi;
      
      qTmp[3]= qf.w();
      qTmp[4]= qf.x();
      qTmp[5]= qf.y();
      qTmp[6]= qf.z();
      if (problemSolver_->problem ()->configValidations()->validate(qTmp,report)) {
        hppDout (info, "waypoint with alpha orientation is valid");
        waypoints[waypoints.size () - 1] = qTmp;
      } else {
        hppDout (info, "waypoint with alpha orientation is NOT valid= " << displayConfig(qTmp));
      }
    }
      // end test Pierre

    // now that the correct orientaion is set, we try to set the trunk as close as possible as the obstacle : 
    if(getCloseToContact){
        Eigen::Vector3d normal;
        for(std::size_t i = 0 ; i< waypoints.size() ; i++ ){
          normal = Eigen::Vector3d(waypoints [i][index],waypoints [i][index+1],waypoints [i][index+2]);
          normal = normal*0.01;
          hppDout(notice,"Direction of motion for getting close to contact :"<<normal[0] << " , "<<normal[1] << " , "<<normal[2]);
          qTmp = waypoints[i];
          qTmp[0] = qTmp[0] - normal[0];
          qTmp[1] = qTmp[1] - normal[1];
          qTmp[2] = qTmp[2] - normal[2];
          while(problemSolver_->problem ()->configValidations()->validate(qTmp,report)){
            qTmp[0] = qTmp[0] - normal[0];
            qTmp[1] = qTmp[1] - normal[1];
            qTmp[2] = qTmp[2] - normal[2];
            hppDout(notice,"new config =  :"<<displayConfig(qTmp));          
          }
          waypoints[i][0] = qTmp[0] + normal[0];
          waypoints[i][1] = qTmp[1] + normal[1];
          waypoints[i][2] = qTmp[2] + normal[2];
        }
    }

	  // loop to construct new path vector with parabPath constructor
    ParabolaPathPtr_t pp;
	  for (std::size_t i = 0; i < num_subpaths; i++) {
	    const core::PathPtr_t subpath = (*path).pathAtRank (i);
	    pp =  boost::dynamic_pointer_cast<ParabolaPath>(subpath);
      if(!pp){
        const core::PathVectorPtr_t pv = boost::dynamic_pointer_cast<core::PathVector>(subpath);
        const core::PathPtr_t subPathEdge = (*pv).pathAtRank (0);
        pp = boost::dynamic_pointer_cast<ParabolaPath>(subPathEdge);
      }
	    const vector_t coefs = pp->coefficients ();
	    const value_type length = pp->length ();



	    if (fullbody) {
	      newPath->appendPath
		(rbprm::BallisticPath::create (robot, waypoints [i],
					       waypoints [i+1],length, coefs));
	    } else { // rbprm
	      newPath->appendPath
		(rbprm::ParabolaPath::create (robot, waypoints [i],
					      waypoints [i+1], length, coefs,
					      pp->V0_, pp->Vimp_,
					      pp->initialROMnames_,
					      pp->endROMnames_,
					      pp->contactCones0_,
					      pp->contactConesImp_));
	    }
	  }
	  // add path vector to problemSolver
	  problemSolver_->addPath (newPath);
	} catch(std::runtime_error& e) {
	    throw Error(e.what());
	  }
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::setFullOrientationMode
      (const bool fullOrientationMode) {
	bindShooter_.fullOrientationMode_ = fullOrientationMode;
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::setContactSize (const double xValue,
					 const double yValue) {
	const core::DevicePtr_t robot = problemSolver_->robot ();
	model::RbPrmDevicePtr_t rbRobot = boost::dynamic_pointer_cast<model::RbPrmDevice>(robot);
	rbRobot->contactSize_ [0] = xValue;
	rbRobot->contactSize_ [1] = yValue;
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::setPose (const hpp::floatSeq& dofArray,
				  const char* poseQuery) throw (hpp::Error){
	std::string query_str = std::string(poseQuery);
	if(query_str.compare ("extending") != 0 && 
	   query_str.compare ("flexion") != 0 && 
	   query_str.compare ("takeoffContact") != 0 &&
	   query_str.compare ("landingContact") != 0)
	  throw std::runtime_error ("Query problem, ask for extending, flexion, takeoffContact or landingContact");
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	if (query_str.compare ("extending") == 0) {
	  extendingPose_ = q;
	  hppDout (info, "extendingPose_= " << displayConfig(extendingPose_));
	}
	else if (query_str.compare ("flexion") == 0){
	  flexionPose_ = q;
	  hppDout (info, "flexionPose_= " << displayConfig(flexionPose_));
	}else if (query_str.compare ("takeoffContact") == 0){
	  takeoffContactPose_ = q;
	  hppDout (info, "takeoffContact= " << displayConfig(takeoffContactPose_));
	}else if (query_str.compare ("landingContact") == 0){
	  landingContactPose_ = q;
	  hppDout (info, "landingContactPose= " << displayConfig(landingContactPose_));
	}
      }

      // ---------------------------------------------------------------

      void RbprmBuilder::setMaxTakeoffVelocity (const double vmax)
	throw (hpp::Error)
      {
	problemSolver_->problem ()->vmaxTakeoff_ = vmax;
      }

      // ---------------------------------------------------------------

      void RbprmBuilder::setMaxLandingVelocity (const double vmax)
	throw (hpp::Error)
      {
	problemSolver_->problem ()->vmaxLanding_ = vmax;
      }

      // ---------------------------------------------------------------

      void RbprmBuilder::setFrictionCoef (const double mu)
	throw (hpp::Error)
      {
	problemSolver_->problem ()->mu_ = mu;
      }

      // ---------------------------------------------------------------

      hpp::intSeq* RbprmBuilder::getResultValues () throw (hpp::Error)
      {
	unsigned int vectorLength =
	  problemSolver_->problem ()->parabolaResults_.size ();
	hpp::intSeq* resultValues = new hpp::intSeq();
	resultValues->length((CORBA::ULong) vectorLength);
	long result;
	for (unsigned int i=0; i<vectorLength; i++) {
	  result = problemSolver_->problem ()->parabolaResults_ [i];
	  (*resultValues) [i] = result;
	}
	return resultValues;
      }

      
      /*void RbprmBuilder::setReferenceConfig (const hpp::floatSeq& dofArray)throw (hpp::Error){
	core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (), dofArray); 
	bindHeuristic_.setConfig(q);
      }*/
    
      void RbprmBuilder::addRefConfigAnalysis (const hpp::floatSeq& dofArray, const char* name)throw (hpp::Error){
          std::size_t configDim = (std::size_t)dofArray.length();
          model::Configuration_t config (configDim);// config.resize (configDim);
          for (std::size_t iDof = 0; iDof < configDim; iDof++) {
              config [iDof] = (double)dofArray[(_CORBA_ULong)iDof];
          }
          model::Configuration_t weight(configDim);
          for(std::size_t iDof = 0; iDof < configDim; iDof++){
              weight[iDof] = 1.;
          }
          BindAnalysis analysis(problemSolver_);
          analysis.setConfig(config);
          analysis.setWeight(weight);
          std::string sname(name);
          bindAnalysis_[sname]=analysis;
          std::cout<<"add heuristic : "<<sname<<std::endl;
          analysisFactory_->AddAnalysis(sname,
                boost::bind(&BindAnalysis::ReferenceAnalysis, boost::ref(bindAnalysis_[sname]), _1,_2));


         /* bindHeuristics_.setConfig(config);
          std::cout<<"add heuristic : "<<std::string(name)<<std::endl;
          fullBody_->AddHeuristic(std::string(name),
                boost::bind(&BindHeuristic::ReferenceHeuristic, boost::ref(bindHeuristics_), _1,_2,_3));
*/
      }

      void RbprmBuilder::addRefConfigAnalysisWeight (const hpp::floatSeq& dofArray, const char* name,const hpp::floatSeq& weightArray)throw (hpp::Error){
          std::size_t configDim = (std::size_t)dofArray.length();
          model::Configuration_t config (configDim);// config.resize (configDim);
          for (std::size_t iDof = 0; iDof < configDim; iDof++) {
              config [iDof] = (double)dofArray[(_CORBA_ULong)iDof];
          }
          model::Configuration_t weight (configDim);// config.resize (configDim);
          for (std::size_t iDof = 0; iDof < configDim; iDof++) {
              weight [iDof] = (double)weightArray[(_CORBA_ULong)iDof];
          }
          BindAnalysis analysis(problemSolver_);
          analysis.setConfig(config);
          analysis.setWeight(weight);
          std::string sname(name);
          bindAnalysis_[sname]=analysis;
          std::cout<<"add analysis : "<<sname<<std::endl;
          analysisFactory_->AddAnalysis(sname,
                boost::bind(&BindAnalysis::ReferenceAnalysis, boost::ref(bindAnalysis_[sname]), _1,_2));


         /* bindHeuristics_.setConfig(config);
          std::cout<<"add heuristic : "<<std::string(name)<<std::endl;
          fullBody_->AddHeuristic(std::string(name),
                boost::bind(&BindHeuristic::ReferenceHeuristic, boost::ref(bindHeuristics_), _1,_2,_3));
*/
      }
    
      // ---------------------------------------------------------------

      void RbprmBuilder::setFullbodyFrictionCoef (const double mu)
      {
	fullBody_->mu_ = mu;
      }


      // ---------------------------------------------------------------

      hpp::floatSeq* RbprmBuilder::convexConePlaneIntersection
      (const unsigned short Ncones, const hpp::floatSeqSeq& cones,
       const double theta, const double mu) throw (hpp::Error){
	std::vector<fcl::Vec3f> cones_vec;
	fcl::Vec3f cone;
	for (unsigned short i = 0; i < Ncones; i++) {
	  for (CORBA::UShort k = 0; k < 3; k++)
	    cone (k) = cones [i][k];
	  cone.normalize ();
	  cones_vec.push_back (cone);
	}
	const core::vector_t CC =
	  convexCone::compute_convex_cone_inter (cones_vec, theta, mu);
	hpp::floatSeq* result = vectorToFloatseq (CC);
	return result;
      }

      // ---------------------------------------------------------------
      hpp::floatSeqSeq* RbprmBuilder::getContactCones
      (const hpp::floatSeq& dofArray, hpp::floatSeqSeq_out conePositionOut)
      throw (hpp::Error) {
	// compute contact-cones DIRECTIONS (not location)
	const core::DevicePtr_t robot = problemSolver_->robot ();
	const core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	library::ContactCones contactCones = library::computeContactCones (problemSolver_->problem(), q);
	const std::vector<fcl::Vec3f> cones = contactCones.directions_;
	const std::vector<fcl::Vec3f> conePositions = contactCones.positions_;

	if (cones.size () != conePositions.size ()) {
	  hppDout (info, "cones.size ()= " << cones.size ());
	  hppDout (info, "conePositions.size ()= " << conePositions.size ());
	  throw Error ("problem with contact-cone positions");
	}
	const std::size_t size = cones.size ();
	hpp::floatSeq* dofArrayCone;
	hpp::floatSeqSeq *conesSequence = new hpp::floatSeqSeq ();
	conesSequence->length ((CORBA::ULong) size);
	hpp::floatSeq* dofArrayPosition;
	hpp::floatSeqSeq *conePositionSeq = new hpp::floatSeqSeq ();
	conePositionSeq->length ((CORBA::ULong) size);
	for (std::size_t i = 0; i < size; i++) {
	  dofArrayCone = vectorToFloatseq (cones [i]);
	  (*conesSequence) [(CORBA::ULong) i] = *dofArrayCone;
	  dofArrayPosition = vectorToFloatseq (conePositions [i]);
	  (*conePositionSeq) [(CORBA::ULong) i] = *dofArrayPosition;
	}
	conePositionOut = conePositionSeq;
	return conesSequence;
      }

      // ---------------------------------------------------------------
      hpp::floatSeqSeq* RbprmBuilder::getlastStatesComputedTime () {
	// get configs and then all times
	std::size_t size = lastStatesComputedTime_.size ();
	vector_t times (size);
	core::Configuration_t q;
	hpp::floatSeq* dofArray;
	hpp::floatSeqSeq *statesTimeSequence;
	statesTimeSequence = new hpp::floatSeqSeq ();
	statesTimeSequence->length ((CORBA::ULong) size + 1);
	for (std::size_t i = 0; i < size; i++) {
	  times [i] = lastStatesComputedTime_ [i].first;
	  // configs
	  q = lastStatesComputedTime_ [i].second.configuration_;
	  dofArray = vectorToFloatseq (q);
	  (*statesTimeSequence) [(CORBA::ULong) i] = *dofArray;
	}
	// time
	dofArray = vectorToFloatseq (times);
	(*statesTimeSequence) [(CORBA::ULong) size] = *dofArray;
	return statesTimeSequence;
      }

      // ---------------------------------------------------------------

      void RbprmBuilder::setFullbodyV0fThetaCoefs (const char* Vquery,
						   const CORBA::Boolean clean,
						   const hpp::floatSeq& Varray,
						   const double theta)
	throw (hpp::Error) {
	std::string Vquery_str = std::string(Vquery);
	if(Vquery_str.compare ("V0") != 0 && Vquery_str.compare ("after") != 0
	   && Vquery_str.compare ("Vimp") != 0
	   && Vquery_str.compare ("before") != 0) {
	  throw std::runtime_error ("Query problem, ask for V0/after or Vimp/before");
	}
	core::vector_t v (3);
	for (std::size_t i = 0; i < 3; i++)
	  v [i] = Varray [(CORBA::ULong) i];
	if (Vquery_str.compare ("V0") == 0 || Vquery_str.compare ("after") ==0){
	  if (!clean) {
	    fullBody_->V0dir_ = v;
	    fullBody_->thetaAfter_ = theta;
	  }
	  else {
	    fullBody_->V0dir_.resize (0);
	    fullBody_->thetaAfter_ = NULL;
	  }
	}
	if (Vquery_str.compare ("Vimp") == 0 || Vquery_str.compare ("before") ==0) {
	  if (!clean) {
	    fullBody_->Vfdir_ = v;
	    fullBody_->thetaBefore_ = theta;
	  }
	  else {
	    fullBody_->Vfdir_.resize (0);
	    fullBody_->thetaBefore_ = NULL;
	  }
	}
      }


      void RbprmBuilder::setFillGenerateContactState (const CORBA::Boolean b) {
	fillGenerateContactState_ = b;
      }

    } // namespace impl
  } // namespace rbprm
} // namespace hpp
