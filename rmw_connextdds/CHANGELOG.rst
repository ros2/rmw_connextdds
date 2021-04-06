^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextdds
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#13 <https://github.com/rticommunity/rmw_connextdds/issues/13>`_ from Ericsson/unique_network_flows
* Refactor common API
* Update branch `master` to support Rolling only (`#15 <https://github.com/rticommunity/rmw_connextdds/issues/15>`_)
* Contributors: Ananya Muddukrishna, Andrea Sorbini, William Woodall

0.4.0 (2021-03-25)
------------------
* Add ability to override of endpoint qos settings based on topic name.
* Optimize QoS for reliable large data.
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
* Renamed environment variables (``RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE``,
  ``RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE``).
* Support a list of initial peers via ``RMW_CONNEXT_INITIAL_PEERS``.

0.2.0 (2021-03-10)
------------------

0.1.1 (2021-03-10)
------------------

0.1.0 (2021-03-10)
------------------
* Initial release.
