#!/usr/bin/env python3
"""Export the EDU actor for the existing A2 C++ ONNX controller."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=ROOT / "networks/01_EDU_A2_STAIR/model_10001.pt",
    )
    parser.add_argument(
        "--policy-dir",
        type=Path,
        default=ROOT / "deploy/robots/a2/config/policy/velocity/v0",
    )
    args = parser.parse_args()

    import onnx
    import torch
    import yaml

    from src.tasks.velocity.config.a2.env_cfgs import POLICY_JOINT_NAMES
    from src.tasks.velocity.config.a2.policy import EduA2Policy

    policy = EduA2Policy(args.checkpoint, device="cpu")
    config_path = args.policy_dir / "params/deploy.yaml"
    config = yaml.safe_load(config_path.read_text())
    output = args.policy_dir / "exported/policy.onnx"
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        policy.actor,
        torch.zeros(1, 225),
        str(output),
        input_names=["obs"],
        output_names=["actions"],
        opset_version=17,
        dynamo=False,
    )
    model = onnx.load(output)
    metadata = {
        "checkpoint_sha256": hashlib.sha256(policy.checkpoint.read_bytes()).hexdigest(),
        "checkpoint_iteration": policy.iteration,
        "policy_joint_names": list(POLICY_JOINT_NAMES),
        "policy_to_sdk_motor_ids": config["joint_ids_map"],
        "input_name": "obs",
        "input_shape": [1, 225],
        "output_name": "actions",
        "output_shape": [1, 12],
        "output_semantics": "raw action in EDU joint order; apply scale/offset/clip in deploy.yaml",
        "history_order": "per term, oldest to newest, 5 frames",
    }
    onnx.helper.set_model_props(model, {k: json.dumps(v) for k, v in metadata.items()})
    onnx.checker.check_model(model)
    onnx.save(model, output)
    metadata["onnx_sha256"] = hashlib.sha256(output.read_bytes()).hexdigest()
    metadata["deploy_yaml_sha256"] = hashlib.sha256(
        config_path.read_bytes()
    ).hexdigest()
    (output.parent / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Exported {output.resolve()} (obs [1, 225] -> actions [1, 12])")


if __name__ == "__main__":
    main()
