# mobman_bt

Behavior-tree-style orchestration for a voice-driven fetch task:

1. Receive `/voice_command` as `std_msgs/String`
2. Search by rotating until `/detected_object_3d` and `/detected_object_label` match
3. Navigate near the object with Nav2 `/navigate_to_pose`
4. Call `/pick_object` as `std_srvs/Trigger`
5. Return to the configured home pose

The included `keyboard_voice_node.py` is a stand-in for speech-to-text. The optional
`speech_voice_node.py` publishes recognized microphone speech to `/voice_command`
when Python `speech_recognition` and microphone support are installed.

For testing without manipulation, launch with `use_mock_pick:=true`.
