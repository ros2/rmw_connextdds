.. _user-build-opts:

Build Options
=============

``rmw_connextdds`` and ``rmw_connextddsmicro`` expose a few compile-time options
that can be passed to ``cmake`` to modify the source code included by each
compilation unit.

You can specify these options to ``colcon build`` using argument ``--cmake-args``,
e.g.:

.. code-block:: sh

    colcon build --cmake-args -DRMW_CONNEXT_LOG_MODE=printf

If you already built your workspace, use option ``--cmake-force-configure`` to
make sure to run ``cmake``'s configure step again (or delete the ``build/``
directory altogether, for a more "drastic" approach).

These options have the same effect for both ``rmw_connextdds``, and
``rmw_connextddsmicro``.

.. note::
  All options are case insentitive.

RMW_CONNEXT_LOG_MODE
--------------------

Option ``RMW_CONNEXT_LOG_MODE`` controls the logging calls made by
``rmw_connextdds``, and ``rmw_connextddsmicro``.

Every log call in the code is wrapped by macros, whose definition can be
altered to select different logging "backends", and to selectively exclude them
from compilation based on the log severity.

default
    Default logging mode, used if ``RMW_CONNEXT_LOG_MODE`` is not specified.

    Use standard ROS 2 logging facilities provided by ``rcutils`` as logging
    backend.
    
    Include log messages at the ``error`` and ``warning`` severities.

all
    Use ``rcutils`` as logging backend. Include log messages of all severities.

    Note that by default, only messages of ``info`` or higher severity will be
    printed. Consult the `ROS 2 documentation <https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html>`_
    for more information on how to increase logging verbosity.

    .. warning::
        This mode should be used with caution, since it will likely cause an
        increase in CPU usage even if messages are not printed to the console.

        Since ``rmw_connextdds`` and ``rmw_connextddsmicro`` do not log any
        messages with ``info`` severity, it is recommended to use mode
        ``printf`` for debugging purposes.

printf
    Use ``printf()`` as logging backend. Include all logging messages.

    This mode will generate **a lot** of output, but it can be useful to
    debug issues in the code with little overhead.

RMW_CONNEXT_WAITSET_MODE
------------------------

Option ``RMW_CONNEXT_WAITSET_MODE`` can be used to select which of the two
available implementations will be used to provision WaitSets, and middleware
Events.

By default, ``rmw_connextdds`` and ``rmw_connextddsmicro`` will use an
implementation based on the standard WaitSet objects included in the DDS API.
While this implementation will report the most accurate event notifications and
always supports all available features, it may also introduce a non-negligible
CPU overhead, which may be undesirable in some scenarios.

For this reason, an alternative more lightweight implementation based on the
standard C++ library is also available. This implementation trades off some of
the event reporting accuracy, for a much lower runtime overhead.

If this implementation is used, the "event status" structures reported by the
RMW will not contain valid "status changes". Applications may only rely on the
"absolute" counters contained in these structures.

dds
    Default implementation based on DDS WaitSets and Conditions.

std
    Alternative implementation based on C++ standard library.
