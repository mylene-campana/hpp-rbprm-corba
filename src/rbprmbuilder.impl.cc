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
#include "rbprmbuilder.impl.hh"
#include "hpp/rbprm/rbprm-device.hh"
#include "hpp/rbprm/rbprm-validation.hh"
#include "hpp/rbprm/interpolation/rbprm-path-interpolation.hh"
#include "hpp/rbprm/interpolation/limb-rrt-helper.hh"
#include "hpp/rbprm/stability/stability.hh"
#include "hpp/rbprm/sampling/sample-db.hh"
#include "hpp/model/urdf/util.hh"
#include <fstream>

#include <hpp/rbprm/fullbodyBallistic/parabola-library.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-path.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-interpolation.hh>
#include "hpp/rbprm/projection-shooter.hh"
#include "hpp/rbprm/planner/parabola-path.hh"
#include <hpp/rbprm/fullbodyBallistic/timed-ballistic-path.hh>


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
    , bindHeuristic_()
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

    model::Configuration_t dofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeq& dofArray)
    {
        std::size_t configDim = (std::size_t)dofArray.length();
        // Get robot
        if (!robot) {
            throw hpp::Error ("No robot in problem solver.");
        }
        std::size_t deviceDim = robot->configSize ();
        // Fill dof vector with dof array.
        model::Configuration_t config (configDim);// config.resize (configDim);
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

    std::vector<model::Configuration_t> doubleDofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        std::size_t configsDim = (std::size_t)doubleDofArray.length();
        std::vector<model::Configuration_t> res;
        for (_CORBA_ULong iConfig = 0; iConfig < configsDim; iConfig++)
        {
            res.push_back(dofArrayToConfig(robot, doubleDofArray[iConfig]));
        }
        return res;
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


    void RbprmBuilder::setNormalFilter(const char* romName, const hpp::floatSeq& normal, double range) throw (hpp::Error)
    {
        std::string name(romName);
        bindShooter_.normalFilter_.erase(name);
        fcl::Vec3f dir;
        for(_CORBA_ULong i =0; i <3; ++i)
        {
            dir[i] = normal[i];
        }
        NormalFilter filter(dir,range);
        bindShooter_.normalFilter_.insert(std::make_pair(name,filter));
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

      hpp::floatSeq* RbprmBuilder::generateContacts(const hpp::floatSeq& configuration, const hpp::floatSeq& direction, const bool noStability) throw (hpp::Error)
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
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
            rbprm::State state = rbprm::ComputeContacts(fullBody_,config,
                                            problemSolver_->collisionObstacles(), dir);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(_CORBA_ULong(state.configuration_.rows()));
            for(std::size_t i=0; i< _CORBA_ULong(config.rows()); i++)
              (*dofArray)[(_CORBA_ULong)i] = state.configuration_ [i];
            return dofArray;
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

    void SetPositionAndNormal(rbprm::State& state, hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs)
    {
        core::Configuration_t old = fullBody->device_->currentConfiguration();
        model::Configuration_t config = dofArrayToConfig (fullBody->device_, configuration);
        fullBody->device_->currentConfiguration(config);
        fullBody->device_->computeForwardKinematics();
        std::vector<std::string> names = stringConversion(contactLimbs);
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
            state.contactNormals_[*cit] = fcl::Vec3f(rot(0,2),rot(1,2), rot(2,2));
            state.contacts_[*cit] = true;
            state.contactOrder_.push(*cit);
        }        
        state.nbContacts = state.contactNormals_.size() ;
        state.configuration_ = config;
        state.robustness =  stability::IsStable(fullBody,state);
        state.stable = state.robustness >= 0;
        fullBody->device_->currentConfiguration(old);
    }

    void RbprmBuilder::setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
            SetPositionAndNormal(startState_,fullBody_, configuration, contactLimbs);
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
            SetPositionAndNormal(endState_,fullBody_, configuration, contactLimbs);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    floatSeqSeq* RbprmBuilder::interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold) throw (hpp::Error)
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
            hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = rbprm::interpolation::RbPrmInterpolation::create(fullBody_,startState_,endState_);
            std::vector<model::Configuration_t> configurations = doubleDofArrayToConfig(fullBody_->device_, configs);
            lastStatesComputed_ = interpolator->Interpolate(problemSolver_->collisionObstacles(),configurations,robustnessTreshold);
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

    floatSeqSeq* RbprmBuilder::interpolate(double timestep, double path, double robustnessTreshold) throw (hpp::Error)
    {
        try
        {
        unsigned int pathId = int(path);
        if(startState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("Start state not initialized, can not interpolate ");
        }
        if(endState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("End state not initialized, can not interpolate ");
        }

        if(problemSolver_->paths().size() <= pathId)
        {
            throw std::runtime_error ("No path computed, cannot interpolate ");
        }

        hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = hpp::rbprm::interpolation::RbPrmInterpolation::create(fullBody_,startState_,endState_,problemSolver_->paths()[pathId]);
        lastStatesComputed_ = interpolator->Interpolate(problemSolver_->collisionObstacles(),timestep,robustnessTreshold);

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

    void RbprmBuilder::interpolateBetweenStates(double state1, double state2) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            //create helper
//            /interpolation::LimbRRTHelper helper(fullBody_, problemSolver_->problem());
            core::PathVectorPtr_t path = interpolation::interpolateStates(fullBody_,problemSolver_->problem(),
                                                                          lastStatesComputed_.begin()+s1,lastStatesComputed_.begin()+s2);
            problemSolver_->addPath(path);
            problemSolver_->robot()->setDimensionExtraConfigSpace(problemSolver_->robot()->extraConfigSpace().dimension()+1);
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
            std::string error("Can not open outfile " + std::string(outfilename));
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
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            std::cout << "name " << * cit << std::endl;
            const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody_->GetLimbs().at(std::string(*cit));
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

    void RbprmBuilder::SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver)
    {
      hppDout (info, "set problem solver");
        problemSolver_ = problemSolver;
        bindShooter_.problemSolver_ = problemSolver;
        bindHeuristic_.problemSolver_ =  problemSolver;
        //bind shooter creator to hide problem as a parameter and respect signature

        // add rbprmshooter
        problemSolver->add<core::ConfigurationShooterBuilder_t>("RbprmShooter",
                                                   boost::bind(&BindShooter::create, boost::ref(bindShooter_), _1));
        problemSolver->add<core::PathValidationBuilder_t>("RbprmPathValidation",
                                                   boost::bind(&BindShooter::createPathValidation, boost::ref(bindShooter_), _1, _2));
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

      // TODO: use ROM while shifting toward obstacle
      hpp::floatSeq* RbprmBuilder::projectOnObstacle
      (const hpp::floatSeq& dofArray, const double dist) throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	core::Configuration_t q_proj;
	model::RbPrmDevicePtr_t rbRobot = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
	std::vector<std::string> filter = bindShooter_.romFilter_;
	const std::map<std::string, rbprm::NormalFilter>& normalFilter = bindShooter_.normalFilter_;
	rbprm::ProjectionShooterPtr_t shooter  = 
	  rbprm::ProjectionShooter::create(robot, *(problemSolver_->problem()),
					   dist, rbRobot, filter, 
					   normalFilter);
	q_proj = shooter->project (q);
	hppDout (info, "q_proj: " << displayConfig (q_proj));

	// Try to rotate the robot manually according to surface normal info
	// a priori non-needed if orientation in projection-shooter
	// orientation done before proj to correct the distance
	//q_proj = rbprm::setOrientation (robot, q_proj);

	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = q_proj (i);
	return dofArray_out;
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
	  const std::vector<std::string>& filter = bindShooter_.romFilter_;
	  const std::map<std::string, rbprm::NormalFilter>& normalFilters = 
	    bindShooter_.normalFilter_;
	  core::ValidationReportPtr_t valReport, unusedreport, valReport2;
	  rbprm::RbPrmValidationPtr_t validator = rbprm::RbPrmValidation::create
	    (rbRobot, filter, normalFilters);
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
						   const double u_offset)
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
	  core::PathVectorPtr_t newPath = core::PathVector::create 
	    (fullBody_->device_->configSize (),
	     fullBody_->device_->numberDof ());
	  hpp::rbprm::BallisticInterpolationPtr_t interpolator = 
	    rbprm::BallisticInterpolation::create(*(problemSolver_->problem ()),
						  fullBody_, startState_,
						  endState_, path);
	  const core::PathPtr_t subpath = (*path).pathAtRank (0);
	  const ParabolaPathPtr_t pp = 
	    boost::dynamic_pointer_cast<ParabolaPath>(subpath);

	  if (extendingPose_.rows() > 0)
	    interpolator->extendingPose (extendingPose_);
	  else
	    hppDout (info, "no extending pose was provided to interpolator");
	  if (flexionPose_.rows() > 0)
	    interpolator->flexionPose (flexionPose_);
	  else
	    hppDout (info, "no flexion pose was provided to interpolator");

	  if (subPathNumber == 1)
	    newPath = interpolator->InterpolateDirectPath(u_offset);
	  else
	    newPath = interpolator->InterpolateFullPath(u_offset);

	  std::size_t newPathId = problemSolver_->addPath (newPath);
	  hppDout (info, "newPath Id is: " << newPathId);
	  normalAvVec_ = interpolator->getnormalAverageVec ();
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
	  const std::size_t num_waypoints  = num_subpaths - 1;
	  bool success;
	  std::vector<core::Configuration_t> configs;
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
	      rbprm::ComputeContacts(fullBody_, q1,
				     problemSolver_->collisionObstacles(), dir);
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
					  const bool fullbody)
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
	  for (std::size_t i = 0; i < waypoints.size () - 1; i++) {
	    // theta_(i,i+1)
	    theta_i = atan2 (waypoints [i+1][1]-waypoints [i][1],
					waypoints [i+1][0]-waypoints [i][0]);
	    //hppDout (info, "theta_i: " << theta_i);
	    waypoints [i][index + 3] = theta_i;
	  }
	  // ! last orientation (qEnd) = last theta_i
	  waypoints [waypoints.size () - 1][index + 3] = theta_i;

	  // update waypoints orientations
	  //const JointPtr_t skullJoint = robot_->getJointByName ("Skull");
	  //const std::size_t rank = skullJoint->rankInConfiguration ();
	  // TODO: if fullbody, use OrientationConstraint for skullJoint ??
	  for (std::size_t i = 0; i < waypoints.size (); i++) {
	    waypoints [i] = rbprm::setOrientation (robot, waypoints [i]);
	    //waypoints [i][rank] = skullJoint->lowerBound (0);
	    hppDout (info, "new wp(i): " << displayConfig (waypoints [i]));
	  }

	  // loop to construct new path vector with parabPath constructor
	  for (std::size_t i = 0; i < num_subpaths; i++) {
	    const core::PathPtr_t subpath = (*path).pathAtRank (i);
	    const ParabolaPathPtr_t pp = 
	    boost::dynamic_pointer_cast<ParabolaPath>(subpath);
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
					      pp->endROMnames_));
	    }
	  }
	  // add path vector to problemSolver
	  problemSolver_->addPath (newPath);
	} catch(std::runtime_error& e) {
	    throw Error(e.what());
	  }
      }
      
      void RbprmBuilder::timeParametrizedPath (const CORBA::UShort pathId) throw (hpp::Error){
        std::size_t pid = (std::size_t) pathId;
        if(problemSolver_->paths().size() <= pid) {
          throw std::runtime_error ("No path computed");
        }
        core::DevicePtr_t robot = problemSolver_->robot ();        
        const core::PathVectorPtr_t path = problemSolver_->paths () [pid];
        core::PathPtr_t tmpPath;
        rbprm::BallisticPathPtr_t castedPath1,castedPath1Max,castedPath2,castedPath2Max;
        core::PathVectorPtr_t newPath = core::PathVector::create (robot->configSize (),
                  robot->numberDof ());
        const std::size_t num_subpaths  = (*path).numberPaths ();
        if(num_subpaths%4 != 0 )
          throw std::runtime_error ("interpolated path doesn't have 4 subpath per parabola");
          
        for (std::size_t i = 0; i < num_subpaths; i = i+4) { // convert each subpath
          tmpPath = (*path).pathAtRank (i);
          castedPath1= boost::dynamic_pointer_cast<rbprm::BallisticPath>(tmpPath);
          if(!castedPath1)
            throw std::runtime_error ("subPath isn't a BallisticPath");
          castedPath1Max= boost::dynamic_pointer_cast<rbprm::BallisticPath>((*path).pathAtRank (i+1));
          castedPath2Max= boost::dynamic_pointer_cast<rbprm::BallisticPath>((*path).pathAtRank (i+2));
          castedPath2= boost::dynamic_pointer_cast<rbprm::BallisticPath>((*path).pathAtRank (i+3));
          

          newPath->appendPath(rbprm::TimedBallisticPath::create(castedPath1,castedPath1Max,castedPath2Max,castedPath2)); 
        }
        
        problemSolver_->addPath (newPath);
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* RbprmBuilder::computeConfigGIWC
      (const hpp::floatSeq& dofArray, const double contactLength,
       const double contactWidth)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	
	core::vector_t contactSize (2);
	contactSize [0] = contactLength;
	contactSize [1] = contactWidth;
	const polytope::ProjectedCone* giwc =
	  rbprm::computeConfigGIWC (problemSolver_->problem (), q, contactSize);
	if (!giwc)
	  throw std::runtime_error ("no GIWC could be computed");
	const core::matrix_t& V = giwc->V; // V-representation
	hppDout (info, "V GIWC rows = " << V.rows ()); // num_contacts * 16
	hppDout (info, "V GIWC cols = " << V.cols ()); // 6
	hppDout (info, "V GIWC = " << V);

	hpp::floatSeq* vArray;
	vArray = new hpp::floatSeq ();
	vArray->length (V.cols ());
	hpp::floatSeqSeq *vSequence;
	vSequence = new hpp::floatSeqSeq ();
	vSequence->length ((CORBA::ULong) V.rows ());
	for (std::size_t i = 0; i < V.rows (); i++) {
	  for (std::size_t k = 0; k < V.cols (); k++) {
	    (*vArray) [(CORBA::ULong) k] = V (i,k);
	  }
	  (*vSequence) [(CORBA::ULong) i] = *vArray;
	}
	return vSequence;
	
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
	   query_str.compare ("flexion") != 0)
	  throw std::runtime_error ("Query problem, ask for extending or flexion");
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	if (query_str.compare ("extending") == 0) {
	  extendingPose_ = q;
	  hppDout (info, "extendingPose_= " << displayConfig(extendingPose_));
	}
	else {
	  flexionPose_ = q;
	  hppDout (info, "flexionPose_= " << displayConfig(flexionPose_));
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

      // ---------------------------------------------------------------

      hpp::floatSeqSeq* RbprmBuilder::getnormalAverageVec ()
	throw (hpp::Error) {
	if(normalAvVec_.size () == 0) {
	  throw std::runtime_error ("No normalAvVec, try to call interpolator");
	}
	
	hpp::floatSeq* vArray;
	vArray = new hpp::floatSeq ();
	vArray->length (3);
	hpp::floatSeqSeq *vSequence;
	vSequence = new hpp::floatSeqSeq ();
	vSequence->length ((CORBA::ULong) normalAvVec_.size ());
	hppDout (info, "normalAvVec_.size= " << normalAvVec_.size ());
	for (std::size_t i = 0; i < normalAvVec_.size (); i++) {
	  for (std::size_t k = 0; k < 3; k++) {
	    (*vArray) [(CORBA::ULong) k] = normalAvVec_ [i][k];
	  }
	  (*vSequence) [(CORBA::ULong) i] = *vArray;
	}
	return vSequence;
      }
      
    void RbprmBuilder::setReferenceConfig (const hpp::floatSeq& dofArray)throw (hpp::Error){
      core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (), dofArray); 
      bindHeuristic_.setConfig(q);
    }
    
    void RbprmBuilder::addRefConfigHeuristic ()throw (hpp::Error){
      fullBody_->AddHeuristic("ReferencePose",
      boost::bind(&BindHeuristic::ReferenceHeuristic, boost::ref(bindHeuristic_), _1,_2,_3));
    }
    

    } // namespace impl
  } // namespace rbprm
} // namespace hpp
