library('ggplot2')

d = read.csv("experiment", header = TRUE)

d_default = d[d$inf_object == 100 & d$cheat_mode == 1 & d$world == "testing_room_with_objects.world" & d$inf_obstacle_unexplored == 20 & d$min_certainty == 1,]

d_agg_nbv_count = aggregate( . ~ method + world + cheat_mode + IoU_threshold + nbv_count, data = d_default, mean)

agg = do.call(data.frame, aggregate(. ~ method + world + cheat_mode + IoU_threshold + nbv_count, data = d_default, FUN = function(x) c(mean = mean(x), count = length(x) ) ) )
agg_relevant = agg[agg$time.count > 2,]



ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
ggsave("recall_vs_nbv_count_scattered.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nof_candidates.mean, y=recall.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Number of candidates", y="Recall")
ggsave("recall_vs_nof_candidates_scattered.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.2,], aes(x=nbv_count, y=recall.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
ggsave("precision_vs_nbv_count_scattered.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nof_candidates.mean, y=precision.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Number of candidates", y="Precision")
ggsave("precision_vs_nof_candidates_scattered.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.2,], aes(x=nbv_count, y=precision.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=time.mean, y=recall.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time (seconds)", y="Recall")+
  ylim(c(0,1))
ggsave("recall_vs_time_scattered.png")


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=time.mean, y=precision.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time (seconds)", y="Precision")+
  ylim(c(0,1))
ggsave("precision_vs_time_scattered.png")


d_default = d[d$inf_object == 100 & d$cheat_mode == 1 & d$world == "testing_room_with_objects2.world" & d$inf_obstacle_unexplored == 20 & d$min_certainty == 1,]

d_agg_nbv_count = aggregate( . ~ method + world + cheat_mode + IoU_threshold + nbv_count, data = d_default, mean)

agg = do.call(data.frame, aggregate(. ~ method + world + cheat_mode + IoU_threshold + nbv_count, data = d_default, FUN = function(x) c(mean = mean(x), count = length(x) ) ) )
agg_relevant = agg[agg$time.count > 2,]


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
ggsave("recall_vs_nbv_count_clustered.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision.mean, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
ggsave("precision_vs_nbv_count_clustered.png")




d_cheat = d[d$inf_object == 100 & d$method == "information_gain" & d$world == "testing_room_with_objects2.world" & d$min_certainty == 1,]
d_cheat$cheat = "on"
d_cheat$cheat[d_cheat$cheat_mode == 0] = "off"

d_agg_nbv_count = aggregate( . ~ method + world + cheat + IoU_threshold + nbv_count, data = d_cheat, mean)

agg = do.call(data.frame, aggregate( . ~ method + world + cheat + IoU_threshold + nbv_count, data = d_cheat, FUN = function(x) c(mean = mean(x), count = length(x))))
agg_relevant = agg[agg$cheat_mode.count > 2,]


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall.mean, group=cheat, colour=cheat)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
ggsave("cheat_mode__recall_vs_nbv_count.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision.mean, group=cheat, colour=cheat)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
ggsave("cheat_mode__precision_vs_nbv_count.png")



d_world = d[d$inf_object == 100 & d$method == "information_gain" & d$cheat_mode == 1 & d$min_certainty == 1,]
d_world$scenario = "objects scattered"
d_world$scenario[d_world$world == "testing_room_with_objects2.world"] = "object clusters"
d_agg_nbv_count = aggregate( . ~ method + scenario + cheat_mode + IoU_threshold + nbv_count, data = d_world, mean)

agg = do.call(data.frame, aggregate( . ~ method + scenario + cheat_mode + IoU_threshold + nbv_count, data = d_world, FUN = function(x) c(mean = mean(x), count = length(x))))
agg_relevant = agg[agg$world.count > 2,]


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall.mean, group=scenario, colour=scenario)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
#ggsave("scenario__recall_vs_nbv_count.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision.mean, group=scenario, colour=scenario)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
#ggsave("scenario__precision_vs_nbv_count.png")


d_inf_gain = d[(d$method == "information_gain" | d$method == "information_gain_with_candidates") & d$cheat_mode == 1 & d$world == "testing_room_with_objects2.world" & d$min_certainty == 1,]
d_inf_gain$setting[d_inf_gain$method == "information_gain_with_candidates" & d_inf_gain$inf_object == 100 & d_inf_gain$inf_obstacle_unexplored == 20] = "everything"
d_inf_gain$setting[d_inf_gain$method == "information_gain" & d_inf_gain$inf_object == 100 & d_inf_gain$inf_obstacle_unexplored == 20] = "obstacles with bonus"
d_inf_gain$setting[d_inf_gain$method == "information_gain" & d_inf_gain$inf_obstacle_unexplored == 0] = "obstacles no bonus"
d_inf_gain$setting[d_inf_gain$method == "information_gain_with_candidates" & d_inf_gain$inf_object == 100 & d_inf_gain$inf_object_unexplored == 0] = "obstacles and objects no boni"
d_inf_gain$setting[d_inf_gain$method == "information_gain_with_candidates" & d_inf_gain$inf_object == 100 & d_inf_gain$inf_obstacle == 0] = "objects with bonus"
d_inf_gain$setting[d_inf_gain$method == "information_gain_with_candidates" & d_inf_gain$inf_object == 50] = "everything with reduced object score"

d_agg_nbv_count = aggregate( . ~ method + world + cheat_mode + IoU_threshold + nbv_count + setting, data = d_inf_gain, mean)

agg = do.call(data.frame, aggregate( . ~ method + world + cheat_mode + IoU_threshold + nbv_count + setting, data = d_inf_gain, FUN = function(x) c(mean = mean(x), count = length(x))))
agg_relevant = agg[agg$time.count > 2,]


ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=recall.mean, group=setting, colour=setting)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
ggsave("setting__recall_vs_nbv_count.png")

ggplot(data=agg_relevant[agg_relevant$IoU_threshold == 0.1,], aes(x=nbv_count, y=precision.mean, group=setting, colour=setting)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
ggsave("setting__precision_vs_nbv_count.png")


d_default = d[d$inf_object == 100 & d$cheat_mode == 1 & d$IoU_threshold == 0.1 & d$world == "testing_room_with_objects.world" & d$inf_obstacle_unexplored == 20 & d$min_certainty == 1,]

agg = do.call(data.frame, aggregate( . ~ method + nbv_count, data = d_default, FUN = function(x) c(mean = mean(x), count = length(x) ) ) )
agg_relevant = agg[agg$time.count > 2,]

ggplot(data=agg_relevant, aes(x=time.mean, y=nbv_count, group=method, colour=method)) +
  geom_line() +
  geom_point()+
  labs(x="Time", y="NBV Count")
ggsave("nbv_count_vs_time.png")


d_default = d[d$inf_object == 100 & d$inf_obstacle_unexplored == 20 & d$cheat_mode == 1 & d$method == "information_gain" & d$world == "testing_room_with_objects.world" & d$IoU_threshold == 0.1,]
d_default$min_certainty = factor(d_default$min_certainty)

agg = do.call(data.frame, aggregate(. ~ method + nbv_count + min_certainty, data = d_default, FUN = function(x) c(mean = mean(x), count = length(x) ) ) )
agg_relevant = agg[agg$time.count > 2,]

#d_default[d_default$min_certainty == 1]$minimal_certainty = "1"


#agg_relevant = agg_relevant[agg_relevant$min_certainty > 1,]

ggplot(data=agg_relevant, aes(x=nbv_count, y=recall.mean, group=min_certainty, colour=min_certainty)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Recall")+
  ylim(c(0,1))
ggsave("min_certainty_recall.png")

ggplot(data=agg_relevant, aes(x=nbv_count, y=precision.mean, group=min_certainty, colour=min_certainty)) +
  geom_line() +
  geom_point()+
  labs(x="NBV Count", y="Precision")+
  ylim(c(0,1))
ggsave("min_certainty_precision.png")
