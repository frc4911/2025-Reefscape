// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import edu.wpi.first.wpilibj.Alert;

public record CameraConfig(
    CameraConstants cameraConstants,
    VisionIOInputsAutoLogged inputs,
    VisionIO visionIO,
    Alert disconnectedAlert) {}
