#include <octomap_server/OctomapServer.h>
#include <pcl/surface/convex_hull.h>

using namespace octomap;
using octomap_msgs::Octomap;

namespace octomap_server
{

	void OctomapServer::processNewCandidate(const KeySet& occupied_cells, const PCLPointCloud& new_candidate)
	{
		uint new_candidate_label = new_candidate.points[0].r + new_candidate.points[0].g;
		std::map<uint, double> label_overlaps;

    ROS_INFO_STREAM("----------");
    ROS_INFO_STREAM("Candidate " << new_candidate_label);

    double candidate_volume = computeSingleCandidateVolume(occupied_cells);
    double total_overlap_volume = 0;

    computeOverlaps(occupied_cells, label_overlaps, total_overlap_volume);

    ROS_INFO_STREAM("Candidate volume: " << candidate_volume);
    ROS_INFO_STREAM("Total overlap volume: " << total_overlap_volume);
    ROS_INFO_STREAM("Total overlap with existing candidates is " << total_overlap_volume / candidate_volume * 100 << " percent.");

    if (total_overlap_volume / candidate_volume < 0.05)
    {
    	m_candidateList.addCandidate(new_candidate_label);
    	ROS_INFO_STREAM("New label " << new_candidate_label <<
    		" (" << (uint)m_candidateList.getColor(new_candidate_label).r << "," << (uint)m_candidateList.getColor(new_candidate_label).g << ")" << " created.");
    }
    else
    {
    	uint computed_candidate_label = computeLabel(label_overlaps);
    	ROS_INFO_STREAM(
	      "Candidate " << new_candidate_label << " will be merged into candidate " << computed_candidate_label <<
	      " (" << (uint)m_candidateList.getColor(computed_candidate_label).r << "," << (uint)m_candidateList.getColor(computed_candidate_label).g << ")");
    	new_candidate_label = computed_candidate_label;
    }
    
    // if (computed_label == 0)
    // {
    // 	ROS_INFO_STREAM("New label " << new_candidate_label << " will be created.");
    // 	m_candidateList.addCandidate(new_candidate_label);
    // }
    // else
    // {
    // 	new_candidate_label = computed_label;
    // }

    insertCandidateIntoOctree(m_octree, &m_candidateList, occupied_cells, new_candidate_label);

    fflush(stdout);
	}

	void OctomapServer::insertCandidateIntoOctree(OcTreeT* octree, CandidateList* list, const octomap::KeySet& occupied_cells, const uint& new_candidate_label)
  {
    octomap::ColorOcTreeNode::Color color = list->getColor(new_candidate_label);

    uint null_nodes = 0, non_null_nodes = 0;

    // Apply label
    for (KeySet::const_iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
    {
      ColorOcTreeNode* node = octree->search(*it, 0);

      octree->updateNode(*it, true);

      if (node != NULL)
      {
        octomap::ColorOcTreeNode::Color currentColor = node->getColor();

        if (color.r == currentColor.r && color.g == currentColor.g)
        {
          node->setColor(currentColor.r, currentColor.g, (uint)currentColor.b + 1);
        }
        else
        {
          if ((uint)currentColor.b > 1)
          {
            node->setColor(currentColor.r, currentColor.g, (uint)currentColor.b - 1);
          }
          else
          {
            node->setColor(color.r, color.g, 0);
          }
        }

        non_null_nodes++;
      }
      else
      {
        ColorOcTreeNode* node = octree->search(*it, 0);
        node->setColor(color);
        null_nodes++;
      }
    }

    ROS_INFO_STREAM("Null nodes: " << null_nodes << "; non-null nodes: " << non_null_nodes);
  }

	uint OctomapServer::computeLabel(const std::map<uint, double>& labels)
	{
	  uint greatest_overlap_label = 0;
	  double greatest_overlap = 0;

	  ROS_INFO_STREAM("Labels found: ");
    for (std::map<uint, double>::const_iterator map_it = labels.begin(), map_end = labels.end(); map_it != map_end; map_it++)
    {
    	uint label = map_it->first;
    	double overlap = map_it->second;

      ROS_INFO_STREAM(
        "Label: " << label << " (" << (uint)m_candidateList.getColor(label).r << "," << (uint)m_candidateList.getColor(label).g << ")" <<
        "; overlap: " << overlap);

      if (label != octomap_server::UNLABELLED && overlap > greatest_overlap)
      {
        greatest_overlap = overlap;
        greatest_overlap_label = label;
      }
    }

	  return greatest_overlap_label;
	}

	uint OctomapServer::computeLabelWithMinCertainty(const std::map<uint, double>& labels, std::map<uint, uint>& certainties, uint min_certainty)
	{
		uint greatest_overlap_label = 0;
	  double greatest_overlap = 0;

	  ROS_INFO_STREAM("Labels found: ");
    for (std::map<uint, double>::const_iterator map_it = labels.begin(), map_end = labels.end(); map_it != map_end; map_it++)
    {
    	uint label = map_it->first;
    	double overlap = map_it->second;

      ROS_INFO_STREAM(
        "Label: " << label << " (" << (uint)m_candidateList.getColor(label).r << "," << (uint)m_candidateList.getColor(label).g << ")" <<
        "; overlap: " << overlap <<
        "; certainty: " << certainties[label]);

      if (label != octomap_server::UNLABELLED && overlap > greatest_overlap && certainties[label] >= min_certainty)
      {
        greatest_overlap = overlap;
        greatest_overlap_label = label;
      }
    }

	  return greatest_overlap_label;
	}

	bool OctomapServer::compareGroundTruthToCandidatesSrv(evaluation::CompareGroundTruthsToProposals::Request& req, evaluation::CompareGroundTruthsToProposals::Response& res)
	{
		ROS_INFO_STREAM("\n");
	  ROS_INFO_STREAM("In CompareGroundTruthsToCandidates service, ground truths given " << req.ground_truths.data.size());

	  uint min_certainty = req.min_certainty.data;
	  ROS_INFO_STREAM("Minimum certainty required for match: " << min_certainty);

	  //DEBUG visualisation
	  // PCLPointCloud visualisation_pc;

	  // Get the volumes of all candidates in m_octree
	  std::map<uint, double> candidate_volumes;
	  std::map<uint, uint> candidate_certainties;
	  computeAllCandidateVolumesAndCertainties(m_octree, &m_candidateList, candidate_volumes, candidate_certainties);
	  res.nof_candidates.data = (uint)candidate_volumes.size();
	  
	  //DEBUG
	  // ROS_INFO_STREAM("Candidate volumes (leaf iterator): ");
	  // for (std::map<uint, double>::iterator cand_it = candidate_volumes.begin(), cand_end = candidate_volumes.end(); cand_it != cand_end; cand_it++)
	  // {
	  // 	ROS_INFO_STREAM("Label: " << cand_it->first << ", volume: " << cand_it->second);
	  // }
	  // ROS_INFO_STREAM("Candidate volumes (tree iterator, checking hasChildren() ): ");
	  // std::map<uint, double> candidate_volumes_treeIt = computeAllCandidateVolumesTreeIt(m_octree, &m_candidateList);
	  // for (std::map<uint, double>::iterator cand_it = candidate_volumes_treeIt.begin(), cand_end = candidate_volumes_treeIt.end(); cand_it != cand_end; cand_it++)
	  // {
	  // 	ROS_INFO_STREAM("Label: " << cand_it->first << ", volume: " << cand_it->second);
	  // }
	  //END DEBUG

	  tf::StampedTransform sensorToWorldTf;
	  std::vector<PCLPointCloud> pcl_pc_list;

	  // Convert the ground truth PCs from ROS message to PCL
	  BOOST_FOREACH(sensor_msgs::PointCloud2& pc_msg, req.ground_truths.data)
	  {
	  	try {
		    m_tfListener.lookupTransform(m_worldFrameId, pc_msg.header.frame_id, pc_msg.header.stamp, sensorToWorldTf);
		  } catch(tf::TransformException& ex){
		    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting evaluation");
		    fflush(stdout);
		    return false;
		  }

	  	PCLPointCloud pcl_pc;
	    fromROSMsg(pc_msg, pcl_pc);

	    pcl_pc_list.push_back(pcl_pc);
	  }

	  // If this is the first evaluation for this run, iterate through the ground truths and insert them into m_gts_octree
	  uint counter = 1;
	  if (!m_groundTruthsIntegrated)
	  {
	  	BOOST_FOREACH(PCLPointCloud pcl_pc, pcl_pc_list)
		  {
		  	m_gtsList.addCandidate(counter);

		  	KeySet free_cells_gts, occupied_cells_gts;
	    	unsigned char* colors_gts = new unsigned char[3];

		  	calculateFreeAndOccupiedCellsFromNonGround(pcl_pc, pointTfToOctomap(sensorToWorldTf.getOrigin()), colors_gts, m_gtsOctree, free_cells_gts, occupied_cells_gts);

		  	insertCandidateIntoOctree(m_gtsOctree, &m_gtsList, occupied_cells_gts, counter);

		  	counter++;
		  }

		  m_groundTruthsIntegrated = true;
	  }

	  // Get the volumes of all ground truths in m_gtsOctree
	  std::map<uint, double> gt_volumes;
	  std::map<uint, uint> gt_certainties;
	  computeAllCandidateVolumesAndCertainties(m_gtsOctree, &m_gtsList, gt_volumes, gt_certainties);

	  //DEBUG
	  // ROS_INFO_STREAM("Ground truth volumes (leaf iterator):");
	  // // for (const auto& label_and_vol : gt_volumes)
	  // for (std::map<uint, double>::iterator gt_it = gt_volumes.begin(), gt_end = gt_volumes.end(); gt_it != gt_end; gt_it++)
	  // {
	  // 	ROS_INFO_STREAM("Label: " << gt_it->first << ", volume: " << gt_it->second);
	  // }
	  // ROS_INFO_STREAM("Ground truth volumes (tree iterator, checking hasChildren() ): ");
	  // std::map<uint, double> gt_volumes_treeIt = computeAllCandidateVolumesTreeIt(m_gtsOctree, &m_gtsList);
	  // for (std::map<uint, double>::iterator gt_it = gt_volumes_treeIt.begin(), gt_end = gt_volumes_treeIt.end(); gt_it != gt_end; gt_it++)
	  // {
	  // 	ROS_INFO_STREAM("Label: " << gt_it->first << ", volume: " << gt_it->second);
	  // }
	  //END DEBUG

	  // Iterate through the ground truths, compare them to the candidates in m_octree and find the
	  // candidate with the greatest overlap.
	  counter = 1;
	  BOOST_FOREACH(PCLPointCloud pcl_pc, pcl_pc_list)
	  { 
	    KeySet free_cells, occupied_cells;
    	unsigned char* colors = new unsigned char[3];

	    calculateFreeAndOccupiedCellsFromNonGround(pcl_pc, pointTfToOctomap(sensorToWorldTf.getOrigin()), colors, m_octree, free_cells, occupied_cells);

	    std::map<uint, double> candidate_overlaps;
	    double total_overlap_volume = 0;

	    ROS_INFO_STREAM("");
	    ROS_INFO_STREAM("Ground truth " << counter << ":");
	    ROS_INFO_STREAM("Ground truth volume: " << gt_volumes[counter]);
	    // ROS_INFO_STREAM("Point cloud size: " << pcl_pc.size());
	    // ROS_INFO_STREAM("occupied_cells.size(): " << occupied_cells.size());

	    computeOverlaps(occupied_cells, candidate_overlaps, total_overlap_volume);

	    uint proposal = computeLabelWithMinCertainty(candidate_overlaps, candidate_certainties, min_certainty);

	    // double convex_hull_overlap = calculateConvexHullOverlap(pcl_pc.makeShared(), proposal);

	    if (proposal == octomap_server::UNLABELLED) ROS_INFO_STREAM("No suitable candidate found");
	    else ROS_INFO_STREAM("Proposal is candidate " << proposal);
	    
	    ROS_INFO_STREAM("Candidate volume: " << candidate_volumes[proposal]);
	    ROS_INFO_STREAM("Overlap: " << candidate_overlaps[proposal]);
	    ROS_INFO_STREAM("Candidate certainty: " << candidate_certainties[proposal]);

	    std_msgs::Float64 overlap;
	    std_msgs::Float64 proposal_vol;
	    std_msgs::Float64 groundtruth_vol;

	    overlap.data = candidate_overlaps[proposal];
	    proposal_vol.data = candidate_volumes[proposal];
	    groundtruth_vol.data = gt_volumes[counter];

	    res.overlaps.push_back(overlap);
	    res.proposal_vol.push_back(proposal_vol);
	    res.groundtruth_vol.push_back(groundtruth_vol);

	    counter++;

	    //DEBUG visualisation
	    // visualisation_pc += pcl_pc;
	  }

	  //DEBUG visualisation
	  // sensor_msgs::PointCloud2 visualisation_msg;
	  // pcl::toROSMsg(visualisation_pc, visualisation_msg);
	  // visualisation_msg.header.frame_id = m_worldFrameId;
	  // m_visualisation_pub.publish(visualisation_msg);

	  return true;
	}

	bool OctomapServer::requestLabelCertaintiesSrv(frontier_exploration::RequestLabelCertainties::Request& req, frontier_exploration::RequestLabelCertainties::Response& res)
	{
		ROS_INFO_STREAM("\n");
	  ROS_INFO_STREAM("In RequestLabelCertainties service");

	  // Check that the input is valid, i.e. we have equal numbers of x and ys
	  // if (req.X.size() != req.Y.size())
	  // {
	  // 	ROS_ERROR_STREAM("Lengths of x (" << req.X.size() << ") and y (" << req.Y.size() << ") do not match, aborting RequestLabelCertainties service.");
	  // 	return false;
	  // }

	  // for (uint i = 0; i < req.X.size(); i++)
	  // {
	  // 	ColorOcTreeNode* node = m_octree->search(req.X[i].data, req.Y[i].data, 0);
	  	
	  // 	std_msgs::UInt8 certainty;
	  	
	  // 	if (node != NULL) certainty.data = (uint)node->getColor().b;
	  // 	else certainty.data = 0;

	  // 	res.certainties.push_back(certainty);
	  // }

	  for (OcTreeT::leaf_iterator leafs_it = m_octree->begin_leafs(), leafs_end = m_octree->end_leafs(); leafs_it != leafs_end; leafs_it++)
	  {
	  	point3d pt = leafs_it.getCoordinate();

	  	// if (pt.z() >= 0 && pt.z() < m_res)
	  	// {
	  		ColorOcTreeNode* leaf_node = m_octree->search(leafs_it.getKey(), 0);

	  		std_msgs::Float32 x;
	  		std_msgs::Float32 y;
	  		std_msgs::UInt8 certs;

	  		x.data = pt.x();
	  		y.data = pt.y();
	  		certs.data = (uint)leaf_node->getColor().b;

	  		res.X.push_back(x);
	  		res.Y.push_back(y);
	  		res.certainties.push_back(certs);
	  	// }
	  }

	  return true;
	}

	void OctomapServer::computeOverlaps(const KeySet& occupied_cells, std::map<uint, double>& labels, double& total_overlap_volume)
	{
	  // labels[octomap_server::UNLABELLED] = 0;
	  
	  // for (KeySet::const_iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
	  // {
	  //   // Find what's at that cell already
	  //   ColorOcTreeNode* node = m_octree->search(*it, 0);

	  //   if (node != NULL && !node->hasChildren())
	  //   {
	  //     // Check the label and increment appropriate counter
	  //     uint existing_label = m_candidateList.getLabel(node->getColor());

	  //     // labels[existing_label]++;
	  //     // ROS_INFO_STREAM("Node size: " << pow(m_octree->getNodeSize(getNodeDepth(*it))), 3);
	  //     labels[existing_label] += pow(m_octree->getNodeSize(getNodeDepth(*it)), 3);
	      
	  //     if (existing_label != octomap_server::UNLABELLED) labelled_nodes++;
	  //     // else labels[octomap_server::UNLABELLED]++;
	  //   }
	  //   else labels[octomap_server::UNLABELLED]++;
	  // }

	  for (OcTreeT::leaf_iterator leafs_it = m_octree->begin_leafs(), leafs_end = m_octree->end_leafs(); leafs_it != leafs_end; leafs_it++)
	  {
	  	octomap::OcTreeKey node_key = leafs_it.getKey();

	  	if (occupied_cells.find(node_key) != occupied_cells.end())
	  	{
	  		ColorOcTreeNode* node = m_octree->search(node_key, 0);

			  if (node != NULL && !node->hasChildren() && m_candidateList.getLabel(node->getColor()) != 0)
		    {
		      // labels[m_candidateList.getLabel(node->getColor())] += pow(m_octree->getNodeSize(getNodeDepth(*it)), 3);
		      labels[m_candidateList.getLabel(node->getColor())] += pow(leafs_it.getSize(), 3);
		      total_overlap_volume += pow(leafs_it.getSize(), 3);
		    }
	  	}
	  }
	}

	double OctomapServer::calculateConvexHullOverlap(const PCLPointCloud::Ptr& ground_truth_pc, uint proposal)
	{
		PCLPointCloud::Ptr proposal_pc(new PCLPointCloud);

		for (OcTreeT::leaf_iterator leafs_it = m_octree->begin_leafs(), leafs_end = m_octree->end_leafs(); leafs_it != leafs_end; leafs_it++)
		{
			octomap::OcTreeKey node_key = leafs_it.getKey();
			
			ColorOcTreeNode* leaf_node = m_octree->search(node_key, 0);

			ROS_INFO_STREAM("Here 1");
			
			if ((uint) leaf_node->getColor().b == proposal)
			{
				uint counter = 0;

				uint node_size = m_octree->getNodeSize(getNodeDepth(node_key));
				point3d node_centre = m_octree->keyToCoord(node_key);

				proposal_pc->push_back(PCLPoint(node_centre.x(), node_centre.y(), node_centre.z()));

				// Calculate corner points of node and add those to proposal_pc
				double offset = node_size/2;
				for (double x = node_centre.x() - offset; x <= node_centre.x() + offset; x += offset * 2)
				{
					for (double y = node_centre.y() - offset; y <= node_centre.y() + offset; y += offset * 2)
					{
						for (double z = node_centre.z() - offset; z <= node_centre.z() + offset; z += offset * 2)
						{
							proposal_pc->push_back(PCLPoint(node_centre.x(), node_centre.y(), node_centre.z()));
							counter++;
							if (counter % 100 == 0) ROS_INFO_STREAM(counter);
						}
					}
				}

				ROS_INFO_STREAM(counter << " points");
			}

			ROS_INFO_STREAM("Here 3");

			// Create convex hulls from proposal_pc and ground_truth
			pcl::ConvexHull<PCLPoint> ground_truth_ch, proposal_ch;
			PCLPointCloud ground_truth_hull_pc, proposal_hull_pc;

			ROS_INFO_STREAM("Here 4");

			ground_truth_ch.setComputeAreaVolume(true);
			proposal_ch.setComputeAreaVolume(true);
			ground_truth_ch.setDimension(3);
			proposal_ch.setDimension(3);
			ground_truth_ch.setInputCloud(ground_truth_pc);
			proposal_ch.setInputCloud(proposal_pc);

			ROS_INFO_STREAM("Here 5");
			
			ground_truth_ch.reconstruct(ground_truth_hull_pc);
			proposal_ch.reconstruct(proposal_hull_pc);

			ROS_INFO_STREAM("Here 6");

			// Get volume of both
			ROS_INFO_STREAM("Volume of ground truth: " << ground_truth_ch.getTotalVolume());
			ROS_INFO_STREAM("Volume of proposal: " << proposal_ch.getTotalVolume());
			
			// Do convex hull overlap filter thing (crop hull)
			// Create convex hull of overlap

		}
	}

	double OctomapServer::computeSingleCandidateVolume (const KeySet& occupied_cells)
	{
		CandidateList candidate_list;
		candidate_list.addCandidate(1);
		
		insertCandidateIntoOctree(m_tempOctree, &candidate_list, occupied_cells, 1);

		ROS_INFO_STREAM("Temp candidate list size: " << candidate_list.getAllLabels().size());

		std::map<uint, double> candidate_volumes;
		std::map<uint, uint> candidate_certainties;
		computeAllCandidateVolumesAndCertainties(m_tempOctree, &candidate_list, candidate_volumes, candidate_certainties);

	  m_tempOctree->clear();

		return candidate_volumes[1];
	}

	void OctomapServer::computeAllCandidateVolumesAndCertainties(
		const OcTreeT* octree, const CandidateList* list, std::map<uint, double>& candidate_volumes, std::map<uint, uint>& candidate_certainties)
	{
	  // std::map<uint, double> labels_counter;

	  for (OcTreeT::leaf_iterator leafs_it = octree->begin_leafs(), leafs_end = octree->end_leafs(); leafs_it != leafs_end; leafs_it++)
	  {
	    ColorOcTreeNode* leaf_node = octree->search(leafs_it.getKey(), 0);
	    
	    uint label = list->getLabel(leaf_node->getColor());
	    uint certainty = leaf_node->getColor().b;

  		candidate_volumes[label] += pow(leafs_it.getSize(), 3);
  		
  		if (certainty > candidate_certainties[label]) candidate_certainties[label] = certainty;
	  }
	}

	std::map<uint, double> OctomapServer::computeAllCandidateVolumesTreeIt(const OcTreeT* octree, const CandidateList* list)
	{
	  std::map<uint, double> labels_counter;

	  for (OcTreeT::tree_iterator tree_it = octree->begin_tree(), tree_end = octree->end_tree(); tree_it != tree_end; tree_it++)
	  {
	    ColorOcTreeNode* tree_node = octree->search(tree_it.getKey(), 0);
	    if (tree_node != NULL && !tree_node->hasChildren())
    	{
    		// labels_counter[list->getLabel(tree_node->getColor())] += pow(octree->getNodeSize(getNodeDepth(tree_it.getKey())), 3);
    		labels_counter[list->getLabel(tree_node->getColor())] += pow(tree_it.getSize(), 3);
    		// if (getNodeDepth(tree_it.getKey()) != octree->getTreeDepth()) ROS_INFO_STREAM("Node level: " << getNodeDepth(tree_it.getKey()));
    	}
	  }

	  return labels_counter;
	}

	/**
	  Copied from https://github.com/OctoMap/octomap/issues/40. Author: GitHub user fmder.
	*/
	double OctomapServer::getNodeDepth(const OcTreeKey& inKey){
	  octomap::OcTreeNode* lOriginNode = m_octree->search(inKey);
	  unsigned int lCount = 0;
	  int i = 1;

	  octomap::OcTreeKey lNextKey(inKey);
	  lNextKey[0] = inKey[0] + i;

	  // Count the number of time we fall on the same node in the positive x direction
	  while(m_octree->search(lNextKey) == lOriginNode){
	      ++lCount;
	      lNextKey[0] = inKey[0] + ++i;
	  }

	  i = 1;
	  lNextKey[0] = inKey[0] - i;

	  // Count the number of time we fall on the same node in the negative x direction
	  while(m_octree->search(lNextKey) == lOriginNode){
	      ++lCount;
	      lNextKey[0] = inKey[0] - ++i;
	  }

	  return m_octree->getTreeDepth() - log2(lCount + 1);
	}
}