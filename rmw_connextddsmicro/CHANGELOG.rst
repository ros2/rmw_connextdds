^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connextddsmicro
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
