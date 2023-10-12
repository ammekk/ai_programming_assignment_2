import heapq
import math
class Anagram:
    num_iterations = 0
    
    class SearchNode:
        def __init__(self, state, g_score, h_score, parent):
            self.state = state
            self.g_score = g_score
            self.h_score = h_score
            self.parent = parent

        def __eq__(self, other):
            return isinstance(other, Anagram.SearchNode) and self.state == other.state
        
        def __hash__(self):
            return hash(self.state)

        def __lt__(self, other):
            return other

    def get_path(self, node, goal):
        if not node.parent:
            return [node.state]
        path = self.get_path(node.parent, goal)
        path.append(node.state)
        return path

    def heuristic_first_attempt(self, state, goal):
        score = 0
        adjacent_goal_tiles = []
        adjacent_state_tiles = []
        for position in range(1, len(state)):
            if state[position] != goal[position]:
                score += 1
        misplaced_tiles = score
        score = 0
        adj_matched = set()
        for i in range(len(goal) - 1):
            adjacent_goal_tiles.append(goal[i] + goal[i + 1])
        for i in range(1, len(state) - 1):
            adjacent_state_tiles.append(state[i] + state[i + 1])  
        for tiles in adjacent_state_tiles:
            matched = False
            for position, goal_tiles in enumerate(adjacent_goal_tiles):
                if tiles == goal_tiles and position not in adj_matched:
                    adj_matched.add(position)
                    matched = True
                    break
            if not matched:
                score += 1
        non_adj_tiles = score
        if misplaced_tiles * 1/5 + non_adj_tiles > 0 and misplaced_tiles * 1/5 + non_adj_tiles * 1/2 < 1:
            return misplaced_tiles * 1/5 + non_adj_tiles  
        return math.floor(misplaced_tiles * 1/5 + non_adj_tiles * 1/2)

    def heuristic(self, state, goal):
        position_dict = {}
        for position, letter in enumerate(goal):
            if letter not in position_dict:
                position_dict[letter] = [position]
            else:
                position_dict[letter].append(position)
        last = position_dict[state[-1]].pop()
        counter = 1
        idx = len(state) - 2
        while(counter < len(state) and position_dict[state[idx]][-1] < last):
            last = position_dict[state[idx]].pop()
            idx -= 1
            counter += 1
        return (len(goal) - counter)

    def anagram_expand(self, state, goal):
        node_list = []
        for pos in range(1, len(state)):  # Create each possible state that can be created from the current one in a single step
            new_state = state[1:pos + 1] + state[0] + state[pos + 1:]
            score = self.heuristic(new_state, goal)
            node_list.append((new_state, score))
        return node_list

    # TO DO: b. Return either the solution as a list of states from start to goal or [] if there is no solution.
    def a_star(self, start, goal, expand):
        initial = self.SearchNode(start, 0, self.heuristic(start, goal), None)
        reached = {initial: initial.g_score + initial.h_score}
        frontier = [(initial.g_score + initial.h_score, initial)]
        while frontier:
            self.num_iterations += 1
            current = heapq.heappop(frontier)[1]
            if current.state == goal:
                return self.get_path(current, goal)
            for child_state, child_h_score in expand(current.state, goal):
                child = self.SearchNode(child_state, current.g_score + 1, child_h_score, current)
                child_f_score = child.g_score + child.h_score
                if child not in reached or reached[child] > child_f_score:
                    reached[child] = child_f_score
                    heapq.heappush(frontier, (child_f_score, child))
        return []

    # Finds a solution, i.e., the set of steps from one word to its anagram
    def solve(self,start, goal):
        self.num_iterations = 0
        if (sorted(start) != sorted(goal)):
            print('This is impossible to solve')
            return "IMPOSSIBLE"
        self.solution = self.a_star(start, goal, self.anagram_expand)
        if not self.solution:
            print('No solution found')
            return "NONE"
        print(str(len(self.solution) - 1) + ' steps from start to goal:')
        for step in self.solution:
            print(step)
        print(str(self.num_iterations) + ' A* iterations were performed to find this solution.')
        return str(self.num_iterations)

if __name__ == '__main__':
    anagram = Anagram()
    anagram.solve('PREDATOR', 'TEARDROP')