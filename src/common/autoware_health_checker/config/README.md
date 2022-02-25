The format of the threshold monitoring setting file is as follows.

[1] You must list all monitored keys.
If you want to know all the keys that can be set, run all the necessary nodes and run the following command.
`rosparam dump dump.yaml /diag_reference`

[2] You must set monitoring target threshold depends on your environment.
Anything key not listed here will be excluded from monitoring.

If you create it based on dump.yaml, don't forget to add the namespace health_checker at the top.

```
health_checker:
  your_monitoring_target1:
    min:
      warn: double_warn_value_threshold
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold
    max:
      warn: double_warn_value_threshold
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold

  your_monitoring_target2:
    min:
      # if you want to use default warn/error/fatal value,
      # please remove warn/error/fatal.
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold

  your_monitoring_target3:
    max: default
      # if you want to use all default value in max/min/rate,
      # please put "default" and remove warn/error/fatal.

  your_monitoring_target4: default
    # if you want to use all default value in the key,
    # please put "default" and remove max/min/rate.
    ...

  topic_rate_topic1:
    pub_node: /your_node_a
    sub_node: /your_node_b
    topic_name: /your_topic
    rate:
      warn: double_warning_rate_threshold
      error: double_error_rate_threshold
      fatal: double_fatal_rate_threshold

  topic_rate_topic2:
    pub_node: /your_node_a
    sub_node: /your_node_b
    topic_name: /your_topic
    rate: default
    ...
```
[3] You can load these threshold online.
`rosparam load this_file`
