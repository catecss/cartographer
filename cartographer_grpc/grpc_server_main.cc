#include <iostream>
#include <memory>
#include <string>

#include "cartographer_grpc/proto/cartographer_service.pb.h"
#include "cartographer_grpc/proto/cartographer_service.grpc.pb.h"

#include <grpc++/grpc++.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using cartographer_grpc::proto::Request;
using cartographer_grpc::proto::Reply;
using cartographer_grpc::proto::Cartographer;

class CartographerServiceImpl final : public Cartographer::Service {
  Status DoNothing(ServerContext* context, const Request* request,
                  Reply* reply) override {
    return Status::OK;
  }
};

void RunServer() {
  std::string server_address("0.0.0.0:50051");
  CartographerServiceImpl service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char** argv) {
  RunServer();

  return 0;
}
