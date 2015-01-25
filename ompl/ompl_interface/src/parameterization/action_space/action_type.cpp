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

#include <moveit/ompl_interface/parameterization/action_space/action_based_state_type.h>

namespace ompl_interface
{

    ActionType::ActionType(ActionBasedStateSpacePtr& space)
        : _space(space)
    {
    }

    ActionType::~ActionType()
    {
    }

    MsgAction::MsgAction(ActionBasedStateSpacePtr& space,
                         const apc_msgs::PrimitivePlan& action)
        : ActionType(space),
          _action()
    {
        // Setup this action.
        setMsg(action);
    }

    // Setup this primitive action.
    void MsgAction::setMsg(const apc_msgs::PrimitivePlan& action)
    {
        // Copy the action over.
        _action = action;

        // TODO Check if this action has a support surface.
        // TODO
    }

    // Get the dimension of this action's manifold.
    int MsgAction::getDimension() const
    {
    }

    // Project the state into the lower dimensional manifold spanned by this primitive action.
    void MsgAction::project(ompl::base::State* x_i,
                            ompl::base::State* x_o) const
    {
    }

    // Returns true if this action can propogate from x_i to x_f.
    bool MsgAction::isUseful(ompl::base::State* x_i,
                             ompl::base::State* x_f) const
    {
    }

    // Propogate the primitive action from x_i to x_f if possible.
    void MsgAction::propogate(ompl::base::State* x_i,
                              ompl::base::State* x_f,
                              ompl::base::State* x_o) const
    {
    }

}
