# test_multi_nodes
Create specified number of nodes to evaluate the connection and data reception between publishers and subscribers in each node.  
`publishers` create specified number of nodes and create a publisher in each node. While a subscriber is connected, the publisher sends 120 messages at a rate of 1 messages every 500ms.
`subscribers` create specified number of nodes and create a subscriber in each node.

## How to build
Use Humble environment as an example

```bash
$ source /opt/ros/humble/setup.bash
$ mkdir -p humble_ws/src && cd humble_ws/src
$ git clone https://github.com/Barry-Xu-2018/test_multi_nodes.git
$ cd ..
$ colcon build --merge-install
```

## How to run

Run `publishers` with 100 nodes
```bash
$ source /Path/To/humble_ws/install/setup.bash
$ ros2 run test_multi_nodes publishers -n 100
```

Run `subscribers` with 100 nodes
```bash
$ source /Path/To/humble_ws/install/setup.bash
$ ros2 run test_multi_nodes subscribers -n 100
```

While subscriber is connected, you will see logs in the terminal which run `publishers`
```
[INFO] [1744617408.033154958] [rclcpp]: /topic_1: subscriber is connected and start to publisher message.
[INFO] [1744617408.033346248] [rclcpp]: /topic_2: subscriber is connected and start to publisher message.
[INFO] [1744617408.033383015] [rclcpp]: /topic_4: subscriber is connected and start to publisher message.
[INFO] [1744617408.033408735] [rclcpp]: /topic_3: subscriber is connected and start to publisher message.
[INFO] [1744617408.033434960] [rclcpp]: /topic_5: subscriber is connected and start to publisher message.
[INFO] [1744617408.033460006] [rclcpp]: /topic_12: subscriber is connected and start to publisher message.
[INFO] [1744617408.033484276] [rclcpp]: /topic_6: subscriber is connected and start to publisher message.
...
```

And while all messages are published, you will see logs in the terminal which run `publishers`
```
...
[INFO] [1744617559.064988749] [node_pub_94]: Node node_pub_94 finished publishing.
[INFO] [1744617559.174372892] [node_pub_95]: Node node_pub_95 finished publishing.
[INFO] [1744617559.283682752] [node_pub_96]: Node node_pub_96 finished publishing.
[INFO] [1744617559.393049359] [node_pub_97]: Node node_pub_97 finished publishing.
[INFO] [1744617559.502381510] [node_pub_98]: Node node_pub_98 finished publishing.
[INFO] [1744617559.611805062] [node_pub_100]: Node node_pub_100 finished publishing.
[INFO] [1744617559.611906427] [node_pub_99]: Node node_pub_99 finished publishing.
```
You can press ctl+c to terminate `publishers`.

In the terminal which run `subscribers`, you will see the log after approximately 2 minutes.
```
...
[INFO] [1744619189.178987629] [node_sub_94]: Node node_sub_94 finished receiving.
[INFO] [1744619189.179007255] [node_sub_95]: Node node_sub_95 finished receiving.
[INFO] [1744619189.179028895] [node_sub_99]: Node node_sub_99 finished receiving.
[INFO] [1744619189.179049839] [node_sub_96]: Node node_sub_96 finished receiving.
[INFO] [1744619189.179071273] [node_sub_98]: Node node_sub_98 finished receiving.
[INFO] [1744619189.451505692] [node_sub_100]: Node node_sub_100 finished receiving.
All nodes finished receiving.
```

If you want to change the number of message sent or the frequency of sending messages, you can change `src/common.hpp`.
