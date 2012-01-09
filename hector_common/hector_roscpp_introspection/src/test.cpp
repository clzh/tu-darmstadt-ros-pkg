//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <introspection/introspection.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/lexical_cast.hpp>

using namespace roscpp_introspection;

void print_introspection(MessagePtr message, const std::string& prefix = "");

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Syntax: roscpp_introspection_test <filename or path>" << std::endl;
    exit(1);
  }

  std::string path(argv[1]);
  load(path);

  ros::Time::init();

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/base_link";
  pose.pose.pose.position.x = 5.4;

  std::cout << std::endl << "This is an instance of geometry_msgs/PoseWithCovarianceStamped: " << std::endl << pose << std::endl;

  std::cout << std::endl << "This is the result of introspection: " << std::endl;
  MessagePtr introspected = introspect(pose);
  print_introspection(introspected);

  std::cout << std::endl << "This is the result of another introspection: " << std::endl;
  V_string fields;
  introspected->getFields(fields, true);
  V_string types;
  introspected->getTypes(types, true);
  std::vector<boost::any> values;
  introspected->getValues(values, true);
  assert(fields.size() == types.size());
  assert(fields.size() == values.size());

  for(size_t i = 0; i < fields.size(); ++i) {
    std::cout << types[i] << "\t" << fields[i] << "\t= " << type(types[i])->as<std::string>(values[i]) << " (" << type(types[i])->getTypeId().name() << ")" << std::endl;
  }

  if (argc >= 3) {
    std::cout << std::endl << "You also gave me '" << argv[2] << "' in the commandline:" << std::endl;
    introspected = messageByDataType(argv[2]);
    if (!introspected) {
      std::cout << "I am sorry, I don't know that message type." << std::endl;
    } else {
      V_string fields, types;
      introspected->getFields(fields, true);
      introspected->getTypes(types, true);
      assert(fields.size() == types.size());
      for(size_t i = 0; i < fields.size(); ++i) {
        std::cout << types[i] << "\t" << fields[i] << std::endl;
      }
    }
  }

  exit(0);
}

void print_introspection(MessagePtr message, const std::string& prefix) {
  if (!message->hasInstance()) {
    std::cout << "No instance!" << std::endl;
  }

  for(Message::const_iterator it = message->begin(); it != message->end(); ++it) {
    FieldPtr field = *it;

    if (field->isMessage()) {
      std::cout << prefix << std::string(field->getDataType()) << " " << std::string(field->getName()) << ":" << std::endl;
      MessagePtr expanded = field->expand();
      if (!expanded) {
        std::cout << "(unknown)" << std::endl;
        continue;
      }
      print_introspection(expanded, prefix + "  ");
      continue;
    }

    std::cout << prefix << std::string(field->getDataType()) << " " << std::string(field->getName()) << " = " << field->as<std::string>() << std::endl;
  }
}
