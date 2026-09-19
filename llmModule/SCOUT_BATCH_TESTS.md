# Scout Module1-3 Batch Instructions

> Historical/development test notes: this document covers only the offline Module1-3 contracts. It
> does not mean that the public default demonstration starts a cloud provider, nor does it constitute
> real curtain-traversal acceptance. The repository root `README.md` defines the current release
> boundary.

The corpus contains 24 instructions:

- 12 positive target and alias cases
- 4 executable constraint cases
- 8 expected Module2 rejection cases

Start Module1, Module2, and Module3 first, using the same `ROS_DOMAIN_ID` in every terminal. Then
load the Scout environment in the publisher terminal:

```bash
cd $SCOUT_WORKSPACE/llmModule
source setup_scout_llm.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
```

List all instructions without publishing:

```bash
python3 scripts/publish_scout_test_instructions.py --list
```

Publish only positive cases:

```bash
python3 scripts/publish_scout_test_instructions.py --category positive
```

Publish selected cases:

```bash
python3 scripts/publish_scout_test_instructions.py \
  --case refrigerator_exact_en \
  --case chair_ambiguous
```

Preview a category without ROS:

```bash
python3 scripts/publish_scout_test_instructions.py \
  --category negative --dry-run
```

The default interval is 35 seconds. This prevents requests from accumulating while Module3 waits
up to 30 seconds for validation dependencies. If the validation planner is ready, a shorter interval
can be selected explicitly:

```bash
python3 scripts/publish_scout_test_instructions.py --interval 5
```

Observe each stage in separate terminals:

```bash
ros2 topic echo /semantic_navigation/intent
ros2 topic echo /semantic_navigation/resolution
ros2 topic echo /semantic_navigation/verification
```

The publisher only sends `std_msgs/msg/String` messages to `/user_instruction`. It does not launch
Module4 or publish execution commands. Expected error codes in the YAML assume Module1 produced the
listed normalized intent. Module3 verification PASS still requires the validation planner, costmap,
keepout mask, and `odom -> base_link` TF.
