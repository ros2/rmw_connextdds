^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextdds_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
