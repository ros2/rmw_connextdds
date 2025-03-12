^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextddsmicro
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.1 (2025-03-12)
-------------------
* Added rmw_event_type_is_supported (`#173 <https://github.com/ros2/rmw_connextdds/issues/173>`_) (`#175 <https://github.com/ros2/rmw_connextdds/issues/175>`_)
* Contributors: mergify[bot]

0.22.0 (2024-04-09)
-------------------

0.21.0 (2024-03-28)
-------------------

0.20.1 (2024-03-09)
-------------------

0.20.0 (2024-01-24)
-------------------

0.19.0 (2023-11-06)
-------------------

0.18.0 (2023-10-04)
-------------------
* Add rmw count clients services impl (`#93 <https://github.com/ros2/rmw_connextdds/issues/93>`_)
* Cleanup context implementation (`#131 <https://github.com/ros2/rmw_connextdds/issues/131>`_)
* Contributors: Chris Lalancette, Minju, Lee

0.17.0 (2023-08-21)
-------------------
* Update to C++17 (`#125 <https://github.com/ros2/rmw_connextdds/issues/125>`_)
* Contributors: Chris Lalancette

0.16.0 (2023-07-11)
-------------------

0.15.1 (2023-05-11)
-------------------

0.15.0 (2023-04-27)
-------------------

0.14.0 (2023-04-12)
-------------------
* Dynamic Subscription (BONUS: Allocators): rmw_connextdds (`#115 <https://github.com/ros2/rmw_connextdds/issues/115>`_)
* Add stubs for new rmw interfaces (`#111 <https://github.com/ros2/rmw_connextdds/issues/111>`_)
* Contributors: methylDragon

0.13.0 (2022-11-02)
-------------------
* Add rmw_get_gid_for_client impl (`#92 <https://github.com/ros2/rmw_connextdds/issues/92>`_)
* Contributors: Brian

0.12.1 (2022-09-13)
-------------------

0.12.0 (2022-05-03)
-------------------
* Switch ROS2 -> ROS 2 everywhere (`#83 <https://github.com/ros2/rmw_connextdds/issues/83>`_)
* Contributors: Chris Lalancette

0.11.1 (2022-04-26)
-------------------

0.11.0 (2022-04-08)
-------------------
* Exclude missing sample info fields when building rmw_connextddsmicro (`#79 <https://github.com/ros2/rmw_connextdds/issues/79>`_)
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

0.8.1 (2021-11-19)
------------------
* Add client/service QoS getters. (`#67 <https://github.com/rticommunity/rmw_connextdds/issues/67>`_)
* Contributors: mauropasse

0.8.0 (2021-09-15)
------------------

0.7.0 (2021-06-04)
------------------
* Add rmw_publisher_wait_for_all_acked support. (`#20 <https://github.com/rticommunity/rmw_connextdds/issues/20>`_)
* Contributors: Barry Xu

0.6.1 (2021-04-26)
------------------

0.6.0 (2021-04-11)
------------------
* Use rmw_qos_profile_unknown when adding entity to graph (`#28 <https://github.com/rticommunity/rmw_connextdds/issues/28>`_)
* Resolve issues identified while investigating `#21 <https://github.com/rticommunity/rmw_connextdds/issues/21>`_ (`#22 <https://github.com/rticommunity/rmw_connextdds/issues/22>`_)
* Use Rolling in README's Quick Start
* Remove commented/unused code
* Avoid topic name validation in get_info functions
* Pass HistoryQosPolicy to graph cache
* Reset error string after looking up type support
* Remove DDS-based WaitSet implementation
* Contributors: Andrea Sorbini, Ivan Santiago Paunovic

0.5.0 (2021-04-06)
------------------
* Merge pull request `#13 <https://github.com/rticommunity/rmw_connextdds/issues/13>`_ from Ericsson/unique_network_flows
* Refactor common API
* Update branch `master` to support Rolling only (`#15 <https://github.com/rticommunity/rmw_connextdds/issues/15>`_)
* Contributors: Ananya Muddukrishna, Andrea Sorbini, William Woodall

0.4.0 (2021-03-25)
------------------
* Only trigger data condition if samples were loaned from reader.
* Alternative WaitSet implementation based on C++ std, selectable at
  compile-time.

0.3.1 (2021-03-15)
------------------

0.3.0 (2021-03-12)
------------------
* Add `<buildtool_export_depend>` for `ament_cmake`.

0.2.1 (2021-03-11)
------------------

0.2.0 (2021-03-10)
------------------

0.1.1 (2021-03-10)
------------------

0.1.0 (2021-03-10)
------------------
* Initial release.
