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

#include "redundancy_group_analyzer/redundancy_group.hpp"

PLUGINLIB_EXPORT_CLASS(redundancy_group_analyzer::RedundancyGroup, diagnostic_aggregator::Analyzer)

namespace redundancy_group_analyzer
{
RedundancyGroup::RedundancyGroup()
: path_(""),
  nice_name_(""),
  analyzer_loader_("diagnostic_aggregator", "diagnostic_aggregator::Analyzer"),
  logger_(rclcpp::get_logger("RedundancyGroup"))
{
}

bool RedundancyGroup::init(
  const std::string & path, const std::string & breadcrumb, const rclcpp::Node::SharedPtr n)
{
  RCLCPP_DEBUG(logger_, "init(%s, %s)", path.c_str(), breadcrumb.c_str());
  bool init_ok = true;
  path_ = path;
  breadcrumb_ = breadcrumb;
  nice_name_ = path;

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n->get_parameters(breadcrumb_, parameters)) {
    RCLCPP_WARN(
      logger_, "Couldn't retrieve parameters for analyzer group '%s', namespace '%s'.",
      breadcrumb_.c_str(), n->get_namespace());
    return false;
  }
  RCLCPP_INFO(
    logger_, "Retrieved %zu parameter(s) for analyzer group with prefix '%s'.", parameters.size(),
    breadcrumb_.c_str());

  std::string ns, an_type, an_path, an_breadcrumb;
  std::shared_ptr<Analyzer> analyzer;
  std::string p_type = breadcrumb_.empty() ? "type" : breadcrumb_ + ".type";
  std::string p_path = breadcrumb_.empty() ? "path" : breadcrumb_ + ".path";

  for (const auto & param : parameters) {
    RCLCPP_DEBUG(
      logger_, "Group '%s' found param: %s : %s", nice_name_.c_str(), param.first.c_str(),
      param.second.value_to_string().c_str());

    if (param.first.compare(p_path) == 0) {
      nice_name_ = param.second.value_to_string();
      RCLCPP_DEBUG(logger_, "Group now with name (path): %s", nice_name_.c_str());
    }

    // Find name of the entity that this parameter belongs to
    int pos = 0;
    if (param.first.substr(0, 10).compare("analyzers.") == 0) {
      pos = 10;
    }
    ns = param.first.substr(0, param.first.find(".", pos));

    if (param.first.compare(ns + ".type") == 0) {
      an_type = param.second.value_to_string();
      RCLCPP_DEBUG(
        logger_, "Group '%s' found analyzer type: %s", nice_name_.c_str(), an_type.c_str());
    }
    if (param.first.compare(ns + ".path") == 0) {
      an_path = param.second.value_to_string();
      RCLCPP_DEBUG(
        logger_, "Group '%s' found analyzer path: %s", nice_name_.c_str(), an_path.c_str());
    }

    if (!ns.empty() && !an_type.empty() && !an_path.empty()) {
      RCLCPP_INFO(
        logger_, "Group '%s', creating %s '%s' (breadcrumb: %s) ...", nice_name_.c_str(),
        an_type.c_str(), an_path.c_str(), ns.c_str());

      try {
        if (!analyzer_loader_.isClassAvailable(an_type)) {
          RCLCPP_WARN(
            logger_, "Unable to find Analyzer class %s. Check that Analyzer is fully declared.",
            an_type.c_str());
        }

        analyzer = analyzer_loader_.createSharedInstance(an_type);
      } catch (const pluginlib::LibraryLoadException & e) {
        RCLCPP_ERROR(
          logger_, "Failed to load analyzer %s, type %s. Caught exception: %s", ns.c_str(),
          an_type.c_str(), e.what());
        auto item = std::make_shared<StatusItem>(ns, "Pluginlib exception loading analyzer");
        aux_items_.push_back(item);
        init_ok = false;
        continue;
      }

      if (!analyzer) {
        RCLCPP_ERROR(
          logger_, "Pluginlib returned a null analyzer for %s, namespace %s.", an_type.c_str(),
          n->get_namespace());
        std::shared_ptr<StatusItem> item(
          new StatusItem(ns, "Pluginlib return NULL Analyzer for " + an_type));
        aux_items_.push_back(item);
        init_ok = false;
        continue;
      }

      if (an_type.compare("diagnostic_aggregator/AnalyzerGroup") == 0) {
        an_path = path + "/" + an_path;
      } else if (an_type.compare("redundancy_group_analyzer/RedundancyGroup") == 0) {
        an_path = path + "/" + an_path;
      } else {
        an_path = path;
      }
      an_breadcrumb = (breadcrumb_.empty() ? ns : breadcrumb_ + "." + ns);
      RCLCPP_DEBUG(
        logger_, "Initializing %s in '%s' (breadcrumb: %s) ...", an_type.c_str(), an_path.c_str(),
        an_breadcrumb.c_str());
      if (!analyzer->init(an_path, an_breadcrumb, n)) {
        RCLCPP_ERROR(
          logger_, "Unable to initialize analyzer NS: %s, type: %s", n->get_namespace(),
          an_type.c_str());
        std::shared_ptr<StatusItem> item(new StatusItem(ns, "Analyzer init failed"));
        aux_items_.push_back(item);
        init_ok = false;
        continue;
      } else {
        this->addAnalyzer(analyzer);
        ns = "";
        an_type = "";
        an_path = "";
      }
    }
  }

  if (analyzers_.size() == 0 && !nice_name_.empty()) {
    init_ok = false;
    RCLCPP_ERROR(logger_, "No analyzers initialized in RedundancyGroup '%s'", n->get_namespace());
  } else {
    RCLCPP_INFO(
      logger_, "Initialized analyzer group '%s' with path '%s' and breadcrumb '%s'.",
      nice_name_.c_str(), path_.c_str(), breadcrumb_.c_str());
  }

  return init_ok;
}

RedundancyGroup::~RedundancyGroup()
{
  RCLCPP_DEBUG(logger_, "destructor");
  analyzers_.clear();
}

bool RedundancyGroup::addAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  RCLCPP_INFO(
    logger_, "Adding analyzer '%s' to group '%s'.", analyzer->getName().c_str(),
    nice_name_.c_str());
  analyzers_.push_back(analyzer);
  return true;
}

bool RedundancyGroup::removeAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  RCLCPP_DEBUG(logger_, "removeAnalyzer()");
  auto it = find(analyzers_.begin(), analyzers_.end(), analyzer);
  if (it != analyzers_.end()) {
    analyzers_.erase(it);
    return true;
  }
  return false;
}

bool RedundancyGroup::match(const std::string & name)
{
  RCLCPP_DEBUG(logger_, "Group '%s' match() %s", nice_name_.c_str(), name.c_str());
  if (analyzers_.size() == 0) {
    RCLCPP_WARN(
      logger_, "Group '%s' doesn't contain any analyzers, can't match.", nice_name_.c_str());
    return false;
  }

  bool match_name = false;

  // First check cache
  if (matched_.count(name)) {
    std::vector<bool> & mtch_vec = matched_[name];
    for (auto i = 0u; i < mtch_vec.size(); ++i) {
      if (mtch_vec[i]) {
        return true;
      }
    }
    return false;
  }

  // Building up cache for each name, which analyzer matches
  matched_[name].resize(analyzers_.size());
  for (auto i = 0u; i < analyzers_.size(); ++i) {
    bool mtch = analyzers_[i]->match(name);
    match_name = mtch || match_name;
    matched_[name].at(i) = mtch;
    if (mtch) {
      RCLCPP_INFO(
        logger_, "Group '%s' has a match with my analyzer '%s'.", nice_name_.c_str(),
        analyzers_[i]->getName().c_str());
    }
  }

  return match_name;
}

void RedundancyGroup::resetMatches()
{
  RCLCPP_DEBUG(logger_, "resetMatches()");
  matched_.clear();
}

bool RedundancyGroup::analyze(const std::shared_ptr<StatusItem> item)
{
  RCLCPP_DEBUG(logger_, "analyze()");
  /* @todo(anordman):assertion ROS_ASSERT_MSG(get_logger(), matched_.count(
      item->getName()), "RedundancyGroup was asked to analyze an item it hadn't matched.");*/

  bool analyzed = false;
  std::vector<bool> & mtch_vec = matched_[item->getName()];
  for (auto i = 0u; i < mtch_vec.size(); ++i) {
    if (mtch_vec[i]) {
      analyzed = analyzers_[i]->analyze(item) || analyzed;
    }
  }

  return analyzed;
}

std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> RedundancyGroup::report()
{
  RCLCPP_DEBUG(logger_, "report()");
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> output;

  auto header_status = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
  header_status->name = path_;
  header_status->level = 0;
  header_status->message = "OK";

  if (analyzers_.size() == 0) {
    header_status->level = 2;
    header_status->message = "No analyzers";
    output.push_back(header_status);

    if (header_status->name == "" || header_status->name == "/") {
      header_status->name = "/RedundancyGroup";
    }

    return output;
  }

  bool all_stale = true;
  header_status->level = 3;

  for (auto j = 0u; j < analyzers_.size(); ++j) {
    std::string path = analyzers_[j]->getPath();
    std::string nice_name = analyzers_[j]->getName();

    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed =
      analyzers_[j]->report();

    // Do not report anything in the header values for analyzers that don't report
    if (processed.empty()) {
      continue;
    }

    // Look through processed data for header, append it to header_status
    // Ex: Look for /Robot/Power and append (Power, OK) to header
    for (auto i = 0u; i < processed.size(); ++i) {
      output.push_back(processed[i]);

      // Add to header status
      if (processed[i]->name == path) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = nice_name;
        kv.value = processed[i]->message;

        all_stale = all_stale && (processed[i]->level == 3);
        header_status->level = std::min(header_status->level, processed[i]->level);
        header_status->values.push_back(kv);
      }
    }
  }

  // Report stale as errors unless all stale
  if (header_status->level == 3 && !all_stale) {
    header_status->level = 2;
  }

  header_status->message = valToMsg(header_status->level);

  if (path_ != "" && path_ != "/") {  // No header if we don't have a base path
    output.push_back(header_status);
  }

  for (auto i = 0u; i < aux_items_.size(); ++i) {
    output.push_back(aux_items_[i]->toStatusMsg(path_, true));
  }

  return output;
}

}  // namespace redundancy_group_analyzer
