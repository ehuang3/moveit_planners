/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_ACTION_SPACE_ACTION_BASED_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_ACTION_SPACE_ACTION_BASED_STATE_SPACE_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/parameterization/action_space/action_based_state_type.h>

namespace ompl_interface
{
    class ActionBasedStateSpace;
    class ActionBasedStateSampler;

    /**< \brief boost::shared_ptr to an action model state sampler. */
    typedef boost::shared_ptr<ActionBasedStateSampler> ActionBasedStateSamplerPtr;

    /**< \brief boost::shared_ptr to a constant action model state sampler. */
    typedef boost::shared_ptr<const ActionBasedStateSampler> ActionBasedStateSamplerConstPtr;

    /**< \brief A action model state sampler */
    class ActionBasedStateSampler : public ompl::base::StateSampler
    {
    public:
        ActionBasedStateSampler(const ActionBasedStateSpace *space,
                                const robot_model::JointModelGroup *group,
                                const robot_model::JointBoundsVector *joint_bounds)
            : ompl::base::StateSampler((ompl::base::StateSpace*) space)
            , joint_model_group_(group)
            , joint_bounds_(joint_bounds)
        {
        }

        virtual void sampleUniform(ompl::base::State *state);

        virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance);

        virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev);

    protected:

        random_numbers::RandomNumberGenerator moveit_rng_;
        const robot_model::JointModelGroup *joint_model_group_;
        const robot_model::JointBoundsVector *joint_bounds_;
    };

    class ActionBasedStateSpace : public ModelBasedStateSpace
    {
    public:

        static const std::string PARAMETERIZATION_TYPE;

        ActionBasedStateSpace(const ModelBasedStateSpaceSpecification &spec);
        virtual ~ActionBasedStateSpace();

        /** @name Functionality specific to state spaces (to be implemented by derived state spaces)
            @{ */

        /** \brief Get the dimension of the space (not the dimension of the surrounding ambient space) */
        virtual unsigned int getDimension() const;

        /** \brief Get the maximum value a call to distance() can return (or an upper bound).
            For unbounded state spaces, this function can return infinity.

            \note Tight upper bounds are preferred because the value of the extent is used in
            the automatic computation of parameters for planning. If the bounds are less tight,
            the automatically computed parameters will be less useful.*/
        virtual double getMaximumExtent() const;

        /** \brief Get a measure of the space (this can be thought of as a generalization of volume) */
        virtual double getMeasure() const;

        /** \brief Bring the state within the bounds of the state space. For unbounded spaces this
            function can be a no-op. */
        virtual void enforceBounds(ompl::base::State *state) const;

        /** \brief Check if a state is inside the bounding box. For unbounded spaces this function
            can always return true. */
        virtual bool satisfiesBounds(const ompl::base::State *state) const;

        /** \brief Set the planning volume for the possible SE2 and/or
         * SE3 components of the state space. */
        virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);

        /** \brief Copy a state to another. The memory of source and destination should NOT overlap.
            \note For more advanced state copying methods (partial copy, for example), see \ref advancedStateCopy. */
        virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;

        /** \brief Computes distance between two states. This function satisfies the properties of a
            metric if isMetricSpace() is true, and its return value will always be between 0 and getMaximumExtent() */
        virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;

        /** \brief Get the number of chars in the serialization of a state in this space */
        virtual unsigned int getSerializationLength() const;

        /** \brief Write the binary representation of \e state to \e serialization */
        virtual void serialize(void *serialization, const ompl::base::State *state) const;

        /** \brief Read the binary representation of a state from \e serialization and write it to \e state */
        virtual void deserialize(ompl::base::State *state, const void *serialization) const;

        /** \brief Checks whether two states are equal */
        virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;

        /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to @e
            to state.  The memory location of @e state is not required to be different from the memory of either @e from
            or @e to. */
        virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;

        /** \brief Allocate an instance of the default uniform state sampler for this space */
        virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;

        /** \brief Allocate an instance of the state sampler for this space. This sampler will be allocated with the
            sampler allocator that was previously specified by setStateSamplerAllocator() or, if no sampler allocator
            was specified, allocDefaultStateSampler() is called */
        // virtual ompl::base::StateSamplerPtr allocStateSampler() const;

        /** \brief Allocate a state that can store a point in the described space */
        virtual ompl::base::State* allocState() const;

        /** \brief Free the memory of the allocated state */
        virtual void freeState(ompl::base::State *state) const;

        /** @} */

        /** @name Functionality specific to accessing real values in a state
            @{ */

        /** \brief Many states contain a number of double values. This function provides a means to get the
            memory address of a double value from state \e state located at position \e index. The first double value
            is returned for \e index = 0. If \e index is too large (does not point to any double values in the state),
            the return value is NULL.

            \note This function does @b not map a state to an array of doubles. There may be components of a state that
            do not correspond to double values and they are 'invisible' to this function. Furthermore, this function is
            @b slow and is not intended for use in the implementation of planners. Ideally, state values should not be
            accessed by index. If accessing of individual state elements is however needed, getValueAddressAtLocation()
            provides a faster implementation. */
        virtual double* getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const;

        /** @} */

        /** @name Debugging tools
            @{ */

        /** \brief Print a state to a stream */
        virtual void printState(const ompl::base::State *state, std::ostream &out) const;

        /** \brief Print the settings for this state space to a stream */
        virtual void printSettings(std::ostream &out) const;

        /** \brief Print the list of registered projections. This function is also called by printSettings() */
        // virtual void printProjections(std::ostream &out) const;

        /** \brief Perform sanity checks for this state space. Throws an exception if failures are found.
            \note This checks if distances are always positive, whether the integration works as expected, etc. */
        // virtual void sanityChecks(double zero, double eps, unsigned int flags) const;

        /** @} */

        /** @name Operations with substates
            @{ */

        /** \brief Allocate a sampler that actually samples only components that are part of \e subspace */
        // virtual ompl::base::StateSamplerPtr allocSubspaceStateSampler(const ompl::base::StateSpace *subspace) const;

        /** \brief Compute the location information for various components of the state space. Either this function or
            setup() must be called before any calls to getValueAddressAtName(), getValueAddressAtLocation() (and other
            functions where those are used). */
        // virtual void computeLocations();

        /** @} */

        /** \brief Perform final setup steps. This function is automatically called by the SpaceInformation. If any
            default projections are to be registered, this call will set them and call their setup() functions. It is
            safe to call this function multiple times. At a subsequent call, projections that have been previously user
            configured are not re-instantiated, but their setup() method is still called. */
        // virtual void setup();

    private:

        /** \brief Compute the number of variables in the state space. */
        unsigned int computeVariableCount(const ModelBasedStateSpaceSpecification& spec);

        /** \brief Compute the number of bytes in the state values array. */
        size_t computeStateValuesSize(const ModelBasedStateSpaceSpecification& spec);

    };

}

#endif
