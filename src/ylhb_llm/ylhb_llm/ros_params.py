from rclpy.parameter import Parameter


def declare_string_array_parameter(node, name: str):
    declared = node.declare_parameter(name, Parameter.Type.STRING_ARRAY)
    if declared.value is not None:
        return declared

    result = node.set_parameters([Parameter(name, Parameter.Type.STRING_ARRAY, [])])[0]
    if not result.successful:
        raise ValueError(f'Failed to initialize string array parameter {name}: {result.reason}')
    return node.get_parameter(name)
