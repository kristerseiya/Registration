// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

class PointCloud;
class Feature;
class RegistrationResult;

/// \class FastGlobalRegistrationOption
///
/// \brief Options for FastGlobalRegistration.
class FastGlobalRegistrationOption {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param division_factor Division factor used for graduated non-convexity.
    /// \param use_absolute_scale Measure distance in absolute scale (1) or in
    /// scale relative to the diameter of the model (0).
    /// \param decrease_mu Set
    /// to `true` to decrease scale mu by division_factor for graduated
    /// non-convexity.
    /// \param maximum_correspondence_distance Maximum
    /// correspondence distance (also see comment of USE_ABSOLUTE_SCALE).
    /// \param iteration_number Maximum number of iterations.
    /// \param tuple_scale Similarity measure used for tuples of feature points.
    /// \param maximum_tuple_count Maximum numer of tuples.
    FastGlobalRegistrationOption(double division_factor = 1.4,
                                 bool use_absolute_scale = false,
                                 bool decrease_mu = true,
                                 double maximum_correspondence_distance = 0.025,
                                 int iteration_number = 64,
                                 double tuple_scale = 0.95,
                                 int maximum_tuple_count = 1000)
        : division_factor_(division_factor),
          use_absolute_scale_(use_absolute_scale),
          decrease_mu_(decrease_mu),
          maximum_correspondence_distance_(maximum_correspondence_distance),
          iteration_number_(iteration_number),
          tuple_scale_(tuple_scale),
          maximum_tuple_count_(maximum_tuple_count) {}
    ~FastGlobalRegistrationOption() {}

public:
    /// Division factor used for graduated non-convexity.
    double division_factor_;
    /// Measure distance in absolute scale (1) or in scale relative to the
    /// diameter of the model (0).
    bool use_absolute_scale_;
    /// Set to `true` to decrease scale mu by division_factor for graduated
    /// non-convexity.
    bool decrease_mu_;
    /// Maximum correspondence distance (also see comment of
    /// USE_ABSOLUTE_SCALE).
    double maximum_correspondence_distance_;
    /// Maximum number of iterations.
    int iteration_number_;
    /// Similarity measure used for tuples of feature points.
    double tuple_scale_;
    /// Maximum number of tuples..
    int maximum_tuple_count_;
};

RegistrationResult FastGlobalRegistration(
        const PointCloud &source,
        const PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        const FastGlobalRegistrationOption &option =
                FastGlobalRegistrationOption());
