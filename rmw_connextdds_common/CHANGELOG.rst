^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextdds_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.1 (2025-03-12)
-------------------
* Added rmw_event_type_is_supported (`#173 <https://github.com/ros2/rmw_connextdds/issues/173>`_) (`#175 <https://github.com/ros2/rmw_connextdds/issues/175>`_)
* Contributors: mergify[bot]

0.22.0 (2024-04-09)
-------------------
* Revert "Mitigate discovery race condition between clients and services (`#132 <https://github.com/ros2/rmw_connextdds/issues/132>`_)" (`#146 <https://github.com/ros2/rmw_connextdds/issues/146>`_)
  This reverts commit 7c95abbfc4559b293ebf5e94e20250bdd99d3ac6.
* Mitigate discovery race condition between clients and services (`#132 <https://github.com/ros2/rmw_connextdds/issues/132>`_)
  * Mitigate discovery race condition between clients and services.
* Add: tracepoint for subscribe serialized_message (`#145 <https://github.com/ros2/rmw_connextdds/issues/145>`_)
  * Add: tracepoint for take_serialized_message
  * Fix: TRACETOOLS_TRACEPOINT args
  * Update rmw_connextdds_common/src/common/rmw_subscription.cpp
  Co-authored-by: Christophe Bedard <bedard.christophe@gmail.com>
* Contributors: Andrea Sorbini, Chris Lalancette, h-suzuki-isp

0.21.0 (2024-03-28)
-------------------
* Support Fast CDR v2 (`#141 <https://github.com/ros2/rmw_connextdds/issues/141>`_)
* Contributors: Miguel Company

0.20.1 (2024-03-09)
-------------------
* Fix the rmw_connextdds_common build with gcc 13.2. (`#142 <https://github.com/ros2/rmw_connextdds/issues/142>`_)
  The most important fix here is to #include <cstdint>,
  but also make sure we #include for all used STL functions.
* Fix basic request reply mapping for ConnextDDS Pro (`#139 <https://github.com/ros2/rmw_connextdds/issues/139>`_)
* Contributors: Chris Lalancette, Christopher Wecht

0.20.0 (2024-01-24)
-------------------
* Add ros2_tracing tracepoints (`#120 <https://github.com/ros2/rmw_connextdds/issues/120>`_)
* Contributors: Christopher Wecht

0.19.0 (2023-11-06)
-------------------
* avoid using dds common public mutex directly (`#134 <https://github.com/ros2/rmw_connextdds/issues/134>`_)
* Fix a couple of warnings pointed out by clang. (`#133 <https://github.com/ros2/rmw_connextdds/issues/133>`_)
* Contributors: Chen Lihui, Chris Lalancette

0.18.0 (2023-10-04)
-------------------
* Add rmw count clients services impl (`#93 <https://github.com/ros2/rmw_connextdds/issues/93>`_)
* Conditional internal API access to support Connext 7+ (`#121 <https://github.com/ros2/rmw_connextdds/issues/121>`_)
* Cleanup context implementation (`#131 <https://github.com/ros2/rmw_connextdds/issues/131>`_)
* Contributors: Andrea Sorbini, Chris Lalancette, Minju, Lee

0.17.0 (2023-08-21)
-------------------
* Fix RMW_Connext_Client::is_service_available for micro (`#130 <https://github.com/ros2/rmw_connextdds/issues/130>`_)
* Update to C++17 (`#125 <https://github.com/ros2/rmw_connextdds/issues/125>`_)
* Pass parameters in the correct order to DDS_DataReader_read in rmw_connextdds_count_unread_samples for micro (`#129 <https://github.com/ros2/rmw_connextdds/issues/129>`_)
* Optimize QoS to improve responsiveness of reliable endpoints (`#26 <https://github.com/ros2/rmw_connextdds/issues/26>`_)
* Clear out errors once we have handled them. (`#126 <https://github.com/ros2/rmw_connextdds/issues/126>`_)
* Contributors: Andrea Sorbini, Chris Lalancette, Christopher Wecht

0.16.0 (2023-07-11)
-------------------
* Add support for listener callbacks (`#76 <https://github.com/ros2/rmw_connextdds/issues/76>`_)
* Contributors: Andrea Sorbini

0.15.1 (2023-05-11)
-------------------

0.15.0 (2023-04-27)
-------------------

0.14.0 (2023-04-12)
-------------------
* [rmw_connextdds] New RMW discovery options (`#108 <https://github.com/ros2/rmw_connextdds/issues/108>`_)
* Call get_type_hash_func (`#113 <https://github.com/ros2/rmw_connextdds/issues/113>`_)
* Type hash distribution during discovery (rep2011) (`#104 <https://github.com/ros2/rmw_connextdds/issues/104>`_)
* Implement matched event (`#101 <https://github.com/ros2/rmw_connextdds/issues/101>`_)
* Add in implementation of inconsistent topic. (`#103 <https://github.com/ros2/rmw_connextdds/issues/103>`_)
* Contributors: Barry Xu, Chris Lalancette, Emerson Knapp, Grey, Michael Carroll

0.13.0 (2022-11-02)
-------------------
* Add rmw_get_gid_for_client impl (`#92 <https://github.com/ros2/rmw_connextdds/issues/92>`_)
* Contributors: Brian

0.12.1 (2022-09-13)
-------------------
* Fix assert statement to allow the seconds field of a DDS_Duration_t to be zero (`#88 <https://github.com/ros2/rmw_connextdds/issues/88>`_)
* Contributors: Michael Jeronimo

0.12.0 (2022-05-03)
-------------------
* Handle 'best_available' QoS policies in common  (`#85 <https://github.com/ros2/rmw_connextdds/issues/85>`_)
* Contributors: Jose Luis Rivero

0.11.1 (2022-04-26)
-------------------
* Resolve build error with RTI Connext DDS 5.3.1 (`#82 <https://github.com/ros2/rmw_connextdds/issues/82>`_)
* Contributors: Andrea Sorbini

0.11.0 (2022-04-08)
-------------------
* Exclude missing sample info fields when building rmw_connextddsmicro (`#79 <https://github.com/ros2/rmw_connextdds/issues/79>`_)
* Properly initialize CDR stream before using it for filtering (`#81 <https://github.com/ros2/rmw_connextdds/issues/81>`_)
* Contributors: Andrea Sorbini

0.10.0 (2022-03-28)
-------------------
* Add support for user-specified content filters (`#68 <https://github.com/ros2/rmw_connextdds/issues/68>`_)
* add stub for content filtered topic (`#77 <https://github.com/ros2/rmw_connextdds/issues/77>`_)
* Add sequence numbers to message info structure (`#74 <https://github.com/ros2/rmw_connextdds/issues/74>`_)
* Contributors: Andrea Sorbini, Chen Lihui, Ivan Santiago Paunovic

0.9.0 (2022-03-01)
------------------
* Add rmw listener apis (`#44 <https://github.com/rticommunity/rmw_connextdds/issues/44>`_)
* Contributors: iRobot ROS

0.8.3 (2022-02-10)
------------------

0.8.2 (2022-01-14)
------------------
* Fix cpplint errors (`#69 <https://github.com/ros2/rmw_connextdds/issues/69>`_)
* Contributors: Jacob Perron

0.8.1 (2021-11-19)
------------------
* Add client/service QoS getters. (`#67 <https://github.com/rticommunity/rmw_connextdds/issues/67>`_)
* Contributors: mauropasse

0.8.0 (2021-09-15)
------------------
* Update rmw_context_impl_t definition (`#65 <https://github.com/ros2/rmw_connextdds/issues/65>`_)
* Use the new rmw_dds_common::get_security_files API (`#61 <https://github.com/ros2/rmw_connextdds/issues/61>`_)
* Contributors: Chris Lalancette, Michel Hidalgo

0.7.0 (2021-06-04)
------------------
* Add rmw_publisher_wait_for_all_acked support. (`#20 <https://github.com/rticommunity/rmw_connextdds/issues/20>`_)
* Support extended signature for `message_type_support_callbacks_t::max_serialized_size()` from `rosidl_typesupport_fastrtps_cpp`. (`#14 <https://github.com/rticommunity/rmw_connextdds/issues/14>`_)
* Update includes after rcutils/get_env.h deprecation. (`#55 <https://github.com/rticommunity/rmw_connextdds/issues/55>`_)
* Always modify UserObjectQosPolicy regardless of override policy. (`#53 <https://github.com/rticommunity/rmw_connextdds/issues/53>`_)
* Improved conversion of time values between ROS and DDS formats. (`#43 <https://github.com/rticommunity/rmw_connextdds/issues/43>`_)
* Allow sharing DomainParticipant with C++ applications. (`#25 <https://github.com/rticommunity/rmw_connextdds/issues/25>`_)
* Add environment variable to control override of DomainParticipantQos. (`#41 <https://github.com/rticommunity/rmw_connextdds/issues/41>`_)
* Contributors: Andrea Sorbini, Barry Xu, Christophe Bedard, Miguel Company

0.6.1 (2021-04-26)
------------------
* Correctly detect empty messages (`#33 <https://github.com/rticommunity/rmw_connextdds/issues/33>`_)
* Contributors: Andrea Sorbini

0.6.0 (2021-04-11)
------------------
* Use rmw_qos_profile_unknown when adding entity to graph (`#28 <https://github.com/rticommunity/rmw_connextdds/issues/28>`_)
* Resolve issues identified while investigating `#21 <https://github.com/rticommunity/rmw_connextdds/issues/21>`_ (`#22 <https://github.com/rticommunity/rmw_connextdds/issues/22>`_)
* Use Rolling in README's Quick Start
* Improved implementation of client::is_service_available for Connext Pro
* Only add request header to typecode with Basic req/rep profile
* Remove commented/unused code
* Avoid topic name validation in get_info functions
* Reduce shutdown period to 10ms
* Pass HistoryQosPolicy to graph cache
* Reset error string after looking up type support
* Remove DDS-based WaitSet implementation
* Contributors: Andrea Sorbini, Ivan Santiago Paunovic

0.5.0 (2021-04-06)
------------------
* Merge pull request `#13 <https://github.com/rticommunity/rmw_connextdds/issues/13>`_ from Ericsson/unique_network_flows
* Remove superfluous header inclusion
* Remove conflicting linkage
* Further remove feature-based exclusion
* Remove feature-based exclusion
* Uncrustify
* Refactor common API
* Include required headers if feature is enabled
* Add conditional compilation support
* Prefer more generic file name
* Restrict unique flow endpoint check to versions beyond Foxy
* Indicate missing support for unique network flows
* Update branch `master` to support Rolling only (`#15 <https://github.com/rticommunity/rmw_connextdds/issues/15>`_)
* Contributors: Ananya Muddukrishna, Andrea Sorbini, William Woodall

0.4.0 (2021-03-25)
------------------
* Add ability to override of endpoint qos settings based on topic name (Pro).
* Optimize QoS for reliable large data (Pro).
* Only trigger data condition if samples were loaned from reader.
* Alternative WaitSet implementation based on C++ std, selectable at
  compile-time.

0.3.1 (2021-03-15)
------------------

0.3.0 (2021-03-12)
------------------
* Add `<buildtool_export_depend>` for `ament_cmake`.
* Use default `dds.transport.UDPv4.builtin.ignore_loopback_interface`.

0.2.1 (2021-03-11)
------------------

0.2.0 (2021-03-10)
------------------

0.1.1 (2021-03-10)
------------------
* Don't log an error on WaitSet::wait() timeout.

0.1.0 (2021-03-10)
------------------
* Initial release.
