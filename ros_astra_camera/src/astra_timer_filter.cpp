/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/astra_timer_filter.h"

#include <algorithm>

namespace astra_wrapper {

AstraTimerFilter::AstraTimerFilter(std::size_t filter_len) : filter_len_(filter_len) {}

AstraTimerFilter::~AstraTimerFilter() = default;

void AstraTimerFilter::addSample(double sample) {
  buffer_.push_back(sample);
  if (buffer_.size() > filter_len_) buffer_.pop_front();
}

double AstraTimerFilter::getMedian() {
  if (!buffer_.empty()) {
    std::deque<double> sort_buffer = buffer_;

    std::sort(sort_buffer.begin(), sort_buffer.end());

    return sort_buffer[sort_buffer.size() / 2];
  } else
    return 0.0;
}

double AstraTimerFilter::getMovingAvg() {
  if (!buffer_.empty()) {
    double sum = 0;

    auto it = buffer_.begin();
    auto it_end = buffer_.end();

    while (it != it_end) {
      sum += *(it++);
    }

    return sum / static_cast<double>(buffer_.size());
  } else
    return 0.0;
}

void AstraTimerFilter::clear() { buffer_.clear(); }

}  // namespace astra_wrapper
