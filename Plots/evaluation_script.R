library('ggplot2')

d = read.csv("experiment", header = TRUE)

d_agg_nbv_count = aggregate( . ~ method + world + cheat_mode + IoU_threshold + nbv_count , data = d, mean)


ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")
ggsave("recall_vs_nbv_count.png")

ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.2,], aes(x=nbv_count, y=recall, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")


ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")
ggsave("precision_vs_nbv_count.png")

ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.2,], aes(x=nbv_count, y=precision, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")


ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.1,], aes(x=time, y=recall, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time (seconds)", y="Recall")
ggsave("recall_vs_nbv_time.png")


ggplot(data=d_agg_nbv_count[d_agg_nbv_count$IoU_threshold == 0.1,], aes(x=time, y=precision, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time (seconds)", y="Precision")
ggsave("precision_vs_time.png")


d$tp_rate = d$

ggplot(data=d_agg_nbv_count, aes(x=, y=, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time (seconds)", y="Precision")


