/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{

/** States of the controller's finite state machine.
 *
 */
namespace states
{

/** Hold posture and check contacts.
 *
 */
struct Initial : State
{
  void configure(const mc_rtc::Configuration & config) override;

  /** Start state.
   *
   */
  void start() override;

  /** Teardown state.
   *
   */
  void teardown() override;

  /** Check transitions at beginning of control cycle.
   *
   */
  bool checkTransitions() override;

  /** Main state function, called if no transition at this cycle.
   *
   */
  void runState() override;

  /** Add "Start standing" transition button to GUI.
   *
   */
  void showStartStandingButton();

  /** Remove "Start standing" transition button from GUI.
   *
   */
  void hideStartStandingButton();

protected:
  /** Reset robot to its initial (half-sitting) configuration.
   *
   * The reason why I do it inside the controller rather than via the current
   * mc_rtc way (switching to half_sitting controller then back to this one)
   * is <https://gite.lirmm.fr/multi-contact/mc_rtc/issues/54>.
   */
  void internalReset();

private:
  bool postureTaskIsActive_; /**< Is the posture task active? */
  bool postureTaskWasActive_; /**< Was the posture task active at previous run()? */
  bool startStandingButton_; /**< Is the "Start standing" button displayed in the GUI? */
  bool startStanding_; /**< Has the user clicked on "Start standing"? */
  bool delayStartStanding_;
  int delayStartStandingCounter_;

  /// @{ config
  bool resetFloatingBaseToPlan_ = true;
  bool updateContactFramesToCurrentSurface_ = true;
  bool resetPosture_ = true;
  bool resetPendulumHeight_ = true;
  /// @}
};

} // namespace states

} // namespace lipm_walking
