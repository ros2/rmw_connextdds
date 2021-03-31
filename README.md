# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains two novel implementations of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer which allow developers to deploy their ROS applications on top of
[RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository provides two RMW packages:

- `rmw_connextdds`

- `rmw_connextddsmicro`

Package `rmw_connextdds` is meant to be a replacement for [`rmw_connext_cpp`](https://github.com/ros2/rmw_connext).
This new implementation resolves several performance issues, and it improves out-of-the-box
wire interoperability with ROS 2 and DDS applications.

For any questions or feedback, feel free to [file an issue](https://github.com/ros2/rmw_connextdds/issues/new/choose),
or reach out to robotics@rti.com.

## Additional Documentation

Please consult [the online manual](https://rmw-connextdds.readthedocs.io/en/latest/)
or generate it locally using [Sphinx](https://www.sphinx-doc.org):

```sh
cd rmw_connextdds/

sphinx-build -b html . build

firefox build/index.html
```
