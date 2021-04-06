^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextdds_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
