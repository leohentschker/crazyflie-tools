% Class to generate a probabalistic roadmap

classdef ProbabalisticRoadmap < VertexArrayMotionPlanningTree
    properties
        default_options
        graph_matrix
        samples
    end
    
    methods

        % set the number of variables we will be working with in our state
        % space
        function obj = ProbabalisticRoadmap(num_vars)
            obj = obj@VertexArrayMotionPlanningTree(num_vars);

        end
        
        % gets a random sample from your sample space
        function sample = get_valid_sample(obj, options)
            
            % initialize the sample to invalid
            sample_valid = false;
            
            % keep generating samples until one is valid
            while ~sample_valid
                sample = obj.randomSample();
                sample_valid = obj.isValid(sample);
            end
        end

        % generates a list of valid samples
        function obj = generate_valid_samples(obj, options)
            
            obj.samples = [];
            for n=1:options.total_samples
                obj.samples = [obj.get_valid_sample(options) obj.samples];
            end
            display(obj.samples);
        end
        
        % constructs a graph of the samples and initializes edges between
        % them
        function obj = create_graph_matrix(obj, options)
            % create the sample graph
            obj.graph_matrix = zeros(length(obj.samples), length(obj.samples));
            
            % go through and add edges between nodes in the graph
            for sample_1_ind=1:length(obj.samples)
                for other_sample_ind=sample_1_ind:length(obj.samples)
                    ind = sub2ind(size(obj.graph_matrix), sample_1_ind, other_sample_ind);
                    obj.graph_matrix(ind) = 3;
                end
            end
            display(obj.graph_matrix);
        end
        
        function obj = prm_construct(obj, options)
            % add in the default options
            options = applyDefaults(options, struct('total_samples', 50));

            % get the samples
            obj = obj.generate_valid_samples(options);
            
            % construct a graph for our samples
            obj = obj.create_graph_matrix(options);

        end
        
        function obj = prm_query(obj, start, goal)
            
            display('PRM QUERY');
        end

        function q = interpolate(obj, q1, q2, interpolation_parameter)
          % q = interpolate(obj, q1, q2, interpolation_parameter) returns the
          % set of 3d points between q1 and q2
          display(q1);
          display(dog);
          q = bsxfun(@times,1-interpolation_parameter,q1) + ...
              bsxfun(@times,interpolation_parameter,q2);
        end
        
        function d = distanceMetric(obj, q1, q_array)
            display(q3);
            d = sqrt(sum(bsxfun(@minus, q1, q_array).^2,1));
        end
    end
end