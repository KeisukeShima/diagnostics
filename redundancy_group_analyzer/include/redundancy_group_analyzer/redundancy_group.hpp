// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REDUNDANCY_GROUP_ANALYZER__REDUNDANCY_GROUP_HPP__
#define REDUNDANCY_GROUP_ANALYZER__REDUNDANCY_GROUP_HPP__

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"


namespace redundancy_group_analyzer
{

using diagnostic_aggregator::Analyzer;
using diagnostic_aggregator::StatusItem;

class RedundancyGroup : public Analyzer
{
public:
  RedundancyGroup();

  virtual ~RedundancyGroup();

  /*!
   *\brief Initialized with base path and namespace.
   *
   * The parameters in its namespace determine the sub-analyzers.
   */
  virtual bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node);

  /**!
   *\brief Add an analyzer to this analyzerGroup
   */
  virtual bool addAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /**!
   *\brief Remove an analyzer from this analyzerGroup
   */
  virtual bool removeAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /*!
   *\brief Match returns true if any sub-analyzers match an item
   */
  virtual bool match(const std::string & name);

  /*!
   *\brief Clear match arrays. Used when analyzers are added or removed
   */
  void resetMatches();

  /*!
   *\brief Analyze returns true if any sub-analyzers will analyze an item
   */
  virtual bool analyze(const std::shared_ptr<StatusItem> item);

  /*!
   *\brief The processed output is the combined output of the sub-analyzers,
   * and the top level status
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

  virtual std::string getPath() const {return path_;}

  virtual std::string getName() const {return nice_name_;}

private:
  std::string path_;
  std::string nice_name_;
  std::string breadcrumb_;

  /*!
   *\brief Loads Analyzer plugins in "analyzers" namespace
   */
  pluginlib::ClassLoader<Analyzer> analyzer_loader_;

  rclcpp::Logger logger_;

  /*!
   *\brief These items store errors, if any, for analyzers that failed to initialize or load
   */
  std::vector<std::shared_ptr<StatusItem>> aux_items_;

  std::vector<std::shared_ptr<Analyzer>> analyzers_;

  /*
   *\brief The map of names to matchings is stored internally.
   */
  std::map<const std::string, std::vector<bool>> matched_;
};
}  // namespace redundancy_group_analyzer

#endif  // REDUNDANCY_GROUP_ANALYZER__REDUNDANCY_GROUP_HPP__
