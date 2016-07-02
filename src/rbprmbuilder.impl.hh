// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
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

#ifndef HPP_RBPRM_CORBA_BUILDER_IMPL_HH
# define HPP_RBPRM_CORBA_BUILDER_IMPL_HH

# include <hpp/core/problem-solver.hh>
# include "rbprmbuilder.hh"
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-shooter.hh>
# include <hpp/rbprm/rbprm-validation.hh>
# include <hpp/rbprm/sampling/analysis.hh>
# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/core/discretized-collision-checking.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/rbprm/rbprm-path-validation.hh>
# include <hpp/core/config-validations.hh>
# include <hpp/model/configuration.hh>
# include <hpp/core/distance.hh>

namespace hpp {
  namespace rbprm {
    namespace impl {
      using CORBA::Short;

    struct BindHeuristic
    {
      BindHeuristic():
        refConfig_(),conf_()
      {
      }
      BindHeuristic(hpp::core::ProblemSolverPtr_t problemSolver):
        refConfig_(),conf_(),problemSolver_(problemSolver)
      {
      }
      
      double ReferenceHeuristic(const sampling::Sample& sample, const Eigen::Vector3d& /*direction*/, const Eigen::Vector3d& /*normal*/)
      {
          // TODO compute distance between refConfig and sample sample.configuration_
        conf_ = sample.configuration_;
        hppDout(info,"reference config = "<<model::displayConfig(refConfig_));
        hppDout(info,"current config   = "<<model::displayConfig(conf_));
        // doesn't work anymore since we only use limb config and not full body
        //core::value_type distance = (*(problemSolver_->problem()->distance())) (conf_,refConfig_);

        //compute distance TODO : improve it
        double distance =0 ;
        double d =0;
        for(size_t i = 0 ; i<conf_.size(); i++){
            d=conf_[i] - refConfig_[i];
            distance += weight_[i] * d*d;
        }


        //return distance*1000. + sample.staticValue_;
        hppDout(info,"heuristic value = "<<distance);
        return -distance;
      }
      
      void setConfig(model::Configuration_t ref){
        refConfig_ = ref;
      }

      void setWeight(model::Configuration_t w){
          weight_ = w;
      }
      
      core::Configuration_t refConfig_;
      core::Configuration_t conf_;
      core::Configuration_t weight_;
      hpp::core::ProblemSolverPtr_t problemSolver_;
      
    };
      
    struct BindShooter
    {
        BindShooter(const std::size_t shootLimit = 10000,
                    const std::size_t displacementLimit = 100)
            : shootLimit_(shootLimit)
            , displacementLimit_(displacementLimit), nbFilterMatch_ (0),
	      fullOrientationMode_ (false)
      {}

        hpp::rbprm::RbPrmShooterPtr_t create (const hpp::model::DevicePtr_t& robot)
        {
	  hppDout (info, "create bindshooter");
	  std::size_t effectiveNbFilterMatch (nbFilterMatch_);
	  if (nbFilterMatch_ == 0)
	    effectiveNbFilterMatch = romFilter_.size ();
	  hppDout (info, "effectiveNbFilterMatch= " << effectiveNbFilterMatch);
	  hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
	  rbprm::RbPrmShooterPtr_t shooter = hpp::rbprm::RbPrmShooter::create
	    (robotcast,problemSolver_->problem ()->collisionObstacles(),romFilter_,normalFilter_,shootLimit_,displacementLimit_,effectiveNbFilterMatch);
	  shooter->fullOrientationMode (fullOrientationMode_);
	  hppDout (info, "fullOrientationMode = " << fullOrientationMode_);
            if(!so3Bounds_.empty())
                shooter->BoundSO3(so3Bounds_);
            return shooter;
        }

      hpp::core::PathValidationPtr_t createPathValidation (const hpp::model::DevicePtr_t& robot, const hpp::model::value_type& val)
      {
	std::size_t effectiveNbFilterMatch (nbFilterMatch_);
	if (nbFilterMatch_ == 0)
	  effectiveNbFilterMatch = romFilter_.size ();
	hppDout (info, "effectiveNbFilterMatch= " << effectiveNbFilterMatch);
            hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
            hpp::rbprm::RbPrmValidationPtr_t validation(hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, normalFilter_,effectiveNbFilterMatch));
            hpp::rbprm::RbPrmPathValidationPtr_t collisionChecking = hpp::rbprm::RbPrmPathValidation::create(robot,val);
            collisionChecking->add (validation);
            problemSolver_->problem()->configValidation(core::ConfigValidations::create ());
            problemSolver_->problem()->configValidations()->add(validation);
            return collisionChecking;
        }

        hpp::core::ProblemSolverPtr_t problemSolver_;
        std::vector<std::string> romFilter_;
        std::map<std::string, NormalFilter> normalFilter_;
        std::size_t shootLimit_;
        std::size_t displacementLimit_;
        std::vector<double> so3Bounds_;
        std::size_t nbFilterMatch_;
        bool fullOrientationMode_;
    };

      // -----------------------------------------------------------------
      class RbprmBuilder : public virtual POA_hpp::corbaserver::rbprm::RbprmBuilder
      {
        public:
        RbprmBuilder ();

        virtual void loadRobotRomModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void loadRobotCompleteModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);


        virtual void loadFullBodyRobot (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void setFilter(const hpp::Names_t& roms) throw (hpp::Error);
        virtual void setNormalFilter(const char* romName, const hpp::floatSeq& normal, double range) throw (hpp::Error);
        virtual void boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error);


        virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual CORBA::UShort getNumSamples(const char* limb) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeNodeIds(const char* limb) throw (hpp::Error);
        virtual double getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error);

        virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction,
						const bool noStability = false)
	  throw (hpp::Error);

        virtual hpp::floatSeq* getContactSamplesIds(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb,
                                                   double octreeNodeId) throw (hpp::Error);

        virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                             unsigned short samples, const char *heuristicName, double resolution, const char *contactType,
                             double disableEffectorCollision) throw (hpp::Error);
        virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues,
                                     double disableEffectorCollision) throw (hpp::Error);

        virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold) throw (hpp::Error);
        virtual void interpolateBetweenStates(double state1, double state2) throw (hpp::Error);
        virtual void saveComputedStates(const char* filepath) throw (hpp::Error);
        virtual void saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error);
        virtual void runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error);

      public:
        void SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver);

	virtual hpp::floatSeq* rbShoot () throw (hpp::Error);
	virtual hpp::floatSeq* projectOnObstacle (const hpp::floatSeq& dofArray,
						  const double dist)
	  throw (hpp::Error);
	virtual hpp::floatSeq* setOrientation (const hpp::floatSeq& dofArray)
	  throw (hpp::Error);
	void setRbShooter () throw (hpp::Error);
	void isRbprmValid (const hpp::floatSeq& dofArray,
			   CORBA::Boolean& trunkValidity,
			   CORBA::Boolean& romValidity,
			   CORBA::String_out report) throw (hpp::Error);
	void interpolateBallisticPath (const CORBA::UShort pathId,
				       const double u_offset)
	  throw (hpp::Error);
	hpp::floatSeqSeq* generateWaypointContacts (CORBA::UShort pathId)
	  throw (hpp::Error);
	virtual hpp::floatSeq* fillConfiguration (const hpp::floatSeq& dofArray,
						  const CORBA::UShort fullSize)
	  throw (hpp::Error);
	void setNumberFilterMatch (const CORBA::UShort nbFilterMatch)
	  throw (hpp::Error);
	hpp::floatSeqSeq* getsubPathsV0Vimp (const char* Vquery,
					     CORBA::UShort pathId)
	  throw (hpp::Error);

	void rotateAlongPath (const CORBA::UShort pathId,
			      const bool fullbody) throw (hpp::Error);
  
  void timeParametrizedPath (const CORBA::UShort pathId) throw (hpp::Error);

	hpp::floatSeqSeq* computeConfigGIWC (const hpp::floatSeq& dofArray,
					     const double contactLength,
					     const double contactWidth);

	void setFullOrientationMode (const bool fullOrientationMode);

	void setContactSize (const double xValue, const double yValue);

	void setPose (const hpp::floatSeq& dofArray,
		      const char* poseQuery) throw (hpp::Error);

	void setMaxTakeoffVelocity (const double vmax) throw (hpp::Error);
	void setMaxLandingVelocity (const double vmax) throw (hpp::Error);
	void setFrictionCoef (const double mu) throw (hpp::Error);
	hpp::intSeq* getResultValues () throw (hpp::Error);
    //void setReferenceConfig (const hpp::floatSeq& dofArray)throw (hpp::Error);
    void addRefConfigHeuristic (const hpp::floatSeq& dofArray, const char* name)throw (hpp::Error);
    void addRefConfigHeuristicWeight (const hpp::floatSeq& dofArray, const char* name,const hpp::floatSeq& weightArray)throw (hpp::Error);

      private:
        /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
        core::ProblemSolverPtr_t problemSolver_;
        model::T_Rom romDevices_;
        rbprm::RbPrmFullBodyPtr_t fullBody_;
        bool romLoaded_;
        bool fullBodyLoaded_;
        BindShooter bindShooter_;
        std::map<std::string,BindHeuristic> bindHeuristics_;
        rbprm::State startState_;
        rbprm::State endState_;
        std::vector<rbprm::State> lastStatesComputed_;
	rbprm::RbPrmShooterPtr_t rbShooter_;
        sampling::AnalysisFactory* analysisFactory_;
	core::Configuration_t extendingPose_;
	core::Configuration_t flexionPose_;
      }; // class RobotBuilder
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
