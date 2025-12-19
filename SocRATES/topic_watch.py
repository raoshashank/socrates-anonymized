# topic_watch.py
import threading, time, queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
class _WatchNode(Node):
    def __init__(self, topic_specs,*,context:Context):
        super().__init__('automation_topic_watch',context=context)
        self._latest = {}
        self._conds = {}
        for tname, (type_str, qos) in topic_specs.items():
            msg_type = get_message(type_str)  # e.g., "std_msgs/msg/String"
            profile = qos or QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._latest[tname] = None
            self._conds[tname] = threading.Condition()
            self.create_subscription(
                msg_type,
                tname,
                lambda msg, tn=tname: self._cb(tn, msg),
                profile
            )

    def _cb(self, topic, msg):
        # if topic == '/robot_description':
        #     print("Received message on", topic)
        #     print(type(msg))
        d = message_to_ordereddict(msg)
        with self._conds[topic]:
            self._latest[topic] = d
            self._conds[topic].notify_all()

    def get_last(self, topic):
        return self._latest.get(topic,None)

    def wait_for(self, topic, predicate, timeout=None):
        deadline = None if timeout is None else time.time() + timeout
        cond = self._conds[topic]
        with cond:
            while True:
                msg = self._latest.get(topic)
                # print("Recieved on", topic,"msg is None:", msg is None)
                # if msg is not None:
                #     print("predicate is true:", predicate(msg))
                if msg is not None and predicate(msg):
                    return msg
                if timeout is None:
                    cond.wait()
                else:
                    remaining = max(0.0, deadline - time.time())
                    if remaining == 0.0:
                        raise TimeoutError(f"Timeout waiting for condition on {topic}")
                    cond.wait(timeout=remaining)

        
class TopicWatcher:
    def __init__(self, topic_types: dict, qos_overrides: dict=None):
        """
        topic_types: { "/topic/name": "pkg/msg/Type", ... }
        qos_overrides: optional { "/topic/name": QoSProfile(...) }
        """
        self._topic_specs = {t: (typ, (qos_overrides or {}).get(t)) for t, typ in topic_types.items()}
        self._started = False
        self._ctx = None
        self._node = None
        self._executor = None
        self._spin_thread = None

    def start(self):
        if self._started:
            return
        # give this watcher its own context so we can init/shutdown per trial
        self._ctx = Context()
        self._ctx.init(args=None)

        self._node = _WatchNode(self._topic_specs, context=self._ctx)
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._started = True

    def _spin(self):
        try:
            while self._ctx.ok():
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            if self._node:
                self._executor.remove_node(self._node)

    def stop(self):
        if not self._started:
            return
        # orderly shutdown this watcher only
        if self._ctx and self._ctx.ok():
            self._ctx.shutdown()
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
        self._node = None
        self._executor = None
        self._ctx = None
        self._started = False
        print("TopicWatcher stopped")

    # Public helpers
    def get_last(self, topic):        
        return self._node.get_last(topic)

    def wait_for(self, topic, pred, timeout_seconds=None):
        return self._node.wait_for(topic, pred, timeout_seconds)
