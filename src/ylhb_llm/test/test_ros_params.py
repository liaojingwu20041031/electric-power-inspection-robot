import unittest

from rclpy.parameter import Parameter

from ylhb_llm.ros_params import declare_string_array_parameter


class FakeNode:
    def __init__(self, declared_parameter=None):
        self.declared = []
        self.set_values = []
        self.parameter = declared_parameter or Parameter(
            'followup_words',
            Parameter.Type.NOT_SET,
            None,
        )

    def declare_parameter(self, name, value):
        self.declared.append((name, value))
        return self.parameter

    def set_parameters(self, parameters):
        self.set_values.extend(parameters)
        self.parameter = parameters[0]

        class Result:
            successful = True
            reason = ''

        return [Result()]

    def get_parameter(self, _name):
        return self.parameter


class RosParamsTest(unittest.TestCase):
    def test_declare_string_array_parameter_initializes_empty_array_default(self):
        node = FakeNode()

        result = declare_string_array_parameter(node, 'followup_words')

        self.assertEqual(result.type_, Parameter.Type.STRING_ARRAY)
        self.assertEqual(result.value, [])
        self.assertEqual(node.declared, [('followup_words', Parameter.Type.STRING_ARRAY)])
        self.assertEqual(len(node.set_values), 1)
        self.assertEqual(node.set_values[0].type_, Parameter.Type.STRING_ARRAY)
        self.assertEqual(node.set_values[0].value, [])

    def test_declare_string_array_parameter_keeps_override_value(self):
        override = Parameter('followup_words', Parameter.Type.STRING_ARRAY, ['确认'])
        node = FakeNode(declared_parameter=override)

        result = declare_string_array_parameter(node, 'followup_words')

        self.assertIs(result, override)
        self.assertEqual(node.set_values, [])


if __name__ == '__main__':
    unittest.main()
