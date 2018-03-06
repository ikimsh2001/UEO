import tensorflow as tf
import numpy as np

#x_test = [[224,356,357,380,422,0,1,1,1,12.65,54.16,-52.96,243,366,428,461,493,0,1,1,1,103.06,4.38,-15.52,248,381,438,465,485,0,1,1,1,68.41,3.44,-2.16,247,369,432,461,486,0,1,1,1,8.12,-60.05,48.01,242,365,421,456,483,0,1,1,1,0.11,-68.25,50.92,241,352,403,444,470,0,1,1,1,-0.17,-64.25,48.23,227,342,353,388,429,0,1,1,1,50.06,-2.93,35.43,217,347,357,387,421,0,0,1,1,66.75,-2.43,22.48,267,400,420,447,465,0,0,1,1,38.16,-2.67,15.7,254,418,430,454,493,0,1,1,1,15.41,-22.18,0]]
gg = np.loadtxt('1010_test_5.csv', delimiter=',', dtype=np.float32)
x_test = gg[:]
nb_classes = 24  # 0 ~ 6

X = tf.placeholder(tf.float32, [None, 60])
Y = tf.placeholder(tf.int32, [None, 1])  # 0 ~ 6
Y_one_hot = tf.one_hot(Y, nb_classes)  # one hot
print("one_hot", Y_one_hot)
Y_one_hot = tf.reshape(Y_one_hot, [-1, nb_classes])
print("reshape", Y_one_hot)

W = tf.Variable(tf.random_normal([60, nb_classes]), name='weight')
b = tf.Variable(tf.random_normal([nb_classes]), name='bias')

# tf.nn.softmax computes softmax activations
# softmax = exp(logits) / reduce_sum(exp(logits), dim)
logits = tf.matmul(X, W) + b
hypothesis = tf.nn.softmax(logits)

# Cross entropy cost/loss
cost_i = tf.nn.softmax_cross_entropy_with_logits(logits=logits,
                                                 labels=Y_one_hot)
cost = tf.reduce_mean(cost_i)
optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.1).minimize(cost)

prediction = tf.argmax(hypothesis, 1)
correct_prediction = tf.equal(prediction, tf.argmax(Y_one_hot, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())

    saver = tf.train.Saver()
    saver.restore(sess, 'save/model')

#    print("Your score will be ", sess.run(prediction, feed_dict={X: x_test}))
#    print(sess.run(hypothesis, feed_dict={X: x_test}))

    a = sess.run(prediction, feed_dict={X: x_test})
    print(a)

    xx = result[int(a)]
    print(xx)
