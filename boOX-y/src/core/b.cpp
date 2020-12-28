/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-124_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* b.cpp / 2-124_planck                                                       */
/*----------------------------------------------------------------------------*/
				const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
				for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
				{
				    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
				    {
					sDouble bypass_transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
					if (bypass_transition_distance <= s_EPSILON)
					{
					    continue;
					}			    					
					sDouble bypass_transition_delta_time = bypass_transition_distance / kruhobot.m_properties.m_linear_velo;
					sDouble bypass_transition_finish_time = front_respectful_transition.m_time + bypass_transition_delta_time;
					sDouble bypass_transition_finish_cost = front_respectful_transition.m_cost + bypass_transition_delta_time;
					sDouble bypass_transition_finish_makespan = front_respectful_transition.m_makespan + bypass_transition_delta_time;
					sDouble bypass_transition_waited = front_respectful_transition.m_waited;

					printf("  b: %d (%.3f)\n", neighbor_location_id, bypass_transition_finish_time);

					RespectfulTransition bypass_respectful_transition(last_transition_id++,
											  bypass_transition_finish_time,
											  bypass_transition_finish_cost,
											  bypass_transition_finish_makespan,
											  bypass_transition_waited,
											  neighbor_location_id,
											  front_respectful_transition.m_trans_id);
				
					bypass_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
					bypass_respectful_transition.m_conflict_fingerprint = next_conflict_fingerprint;
								    
					if (!is_TransitionConflicting(front_respectful_transition.m_location_id,
								      neighbor_location_id,
								      front_respectful_transition.m_time,
								      bypass_transition_finish_time,
								      location_Conflicts,
								      linear_Conflicts,
								      bypass_respectful_transition.m_conflict_fingerprint))
					{
					    printf("    b1\n");
					    if (!is_UnifiedlyVisited(neighbor_location_id, bypass_transition_finish_time, unified_Visits))
					    {
						printf("    b2\n");
						if (front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.find(wait_culprit_conflict_id) == front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end())
						{
						    printf("    b3\n");
						    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(bypass_respectful_transition.m_time);
						    
						    if (respectful_exploration == respectful_Explorations.end())
						    {
							printf("    b4\n");
							RespectfulVisits_umap extended_conflict_Visits;
							RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
						    
							RespectfulVisits_umap::iterator bypass_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																							 bypass_respectful_visit)).first;
							RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan, bypass_respectful_transition));
							bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
						    
							respectful_Explorations.insert(RespectfulExplorations_map::value_type(bypass_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));
						    }
						    else
						    {
							printf("    b5\n");
							RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(bypass_respectful_transition.m_location_id);
							
							if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
							{
							    printf("    b6\n");
							    RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
							    RespectfulVisits_umap::iterator bypass_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																								   bypass_respectful_visit)).first;
							    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																														       bypass_respectful_transition));
							    bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
							}
							else /* visiting next time */
							{
							    printf("    b7\n");
							    sASSERT(next_respectful_visit != respectful_exploration->second.end());
							    
							    if (next_respectful_visit->second.m_time > bypass_respectful_transition.m_time)
							    {
								printf("    b8\n");
								next_respectful_visit->second.m_time = bypass_respectful_transition.m_time;
								next_respectful_visit->second.m_trans_id = bypass_respectful_transition.m_trans_id;
								
								RespectfulTransitions_mmap &respectful_transition_Queue = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint];
								
								respectful_transition_Queue.erase(next_respectful_visit->second.m_queue_iter);
								RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																							    bypass_respectful_transition));
								next_respectful_visit->second.m_queue_iter = queue_iter;
							    }
							}
						    }
						}
					    }
					}
				    }
				}
