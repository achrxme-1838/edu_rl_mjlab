"""Keep local observation wrappers compatible with the pinned mjlab API."""

import inspect
import unittest
from unittest.mock import patch

from mjlab.envs.mdp import observations as upstream
from mjlab_wrapper import mdp


def public_observations():
    return {
        name: value for name, value in vars(upstream).items()
        if not name.startswith("_") and inspect.isfunction(value)
        and value.__module__ == upstream.__name__
    }


class MdpWrapperTest(unittest.TestCase):
    def test_only_observations_are_exported_with_matching_signatures(self):
        originals = public_observations()
        self.assertEqual(set(mdp.__all__), set(originals))
        for name, source in originals.items():
            with self.subTest(term=name):
                wrapped = getattr(mdp, name)
                self.assertIsNot(wrapped, source)
                self.assertEqual(wrapped.__module__, "mjlab_wrapper.mdp.observations")
                self.assertEqual(inspect.signature(wrapped), inspect.signature(source))

    def test_all_observation_arguments_and_results_reach_upstream_unchanged(self):
        for name, source in public_observations().items():
            with self.subTest(term=name):
                wrapped = getattr(mdp, name)
                signature = inspect.signature(source)
                # Distinct values expose swapped/dropped arguments, including
                # optional joint selection and sensor parameters.
                args = {key: object() for key in signature.parameters}
                result = object()
                with patch.object(upstream, name, return_value=result) as call:
                    self.assertIs(wrapped(**args), result)
                    call.assert_called_once()
                    forwarded = signature.bind(*call.call_args.args, **call.call_args.kwargs)
                    self.assertEqual(forwarded.arguments, args)


if __name__ == "__main__":
    unittest.main()
