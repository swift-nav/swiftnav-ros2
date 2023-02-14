/*
 * Copyright (C) 2010-2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#pragma once

#include <publishers/dummy_publisher.h>
#include <list>

class PublisherManager {
 public:
  void add(const PublisherPtr& publisher) { publishers_.push_back(publisher); }

 private:
  std::list<PublisherPtr> publishers_;
};
