#ifndef ROVER_H
#define ROVER_H

#include <utility>
#include <vector>
#include <map>
#include <string>
#include <memory>

using coordinate_t = int32_t;

// Abstract class responsible for sensors.
class Sensor {
public:
    Sensor() = default;
    virtual ~Sensor() = default;

    virtual bool is_safe(coordinate_t, coordinate_t) = 0;
};

using sensors_t = std::vector<std::shared_ptr<Sensor>>;

// An exception that is raised when rover is heading towards a dangerous
// field.
class DangerousField : public std::exception {
public:
    const char *what() const noexcept override {
        return "Dangerous Field";
    }
};

// An exception that is raised when rover has not landed yet and get
// an execute request.
class RoverDidNotLand : public std::exception {
public:
    const char *what() const noexcept override {
        return "Rover did not land";
    }
};

// Class responsible for managing coordinates, adding them etc.
class Coordinates {
protected:
    coordinate_t x, y;

public:
    constexpr Coordinates(coordinate_t x, coordinate_t y) : x(x), y(y) {}
    ~Coordinates() = default;

    void operator+=(const Coordinates &other) {
        x += other.x;
        y += other.y;
    }

    bool is_safe(std::shared_ptr<Sensor> sensor) {
        return sensor->is_safe(x, y);
    }

    friend std::ostream& operator<<(std::ostream& os,
            const Coordinates &coordinates) {
        os << "(" << coordinates.x << ", " << coordinates.y << ")";
        return os;
    }
};

enum class Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Assignment of consts to specific direction.
class DirectionManager {
private:
    constexpr static int DIRECTIONS_NO = 4;
    constexpr static Coordinates direction_move[DIRECTIONS_NO] =
            {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    constexpr static std::string_view direction_name[DIRECTIONS_NO] =
            {"NORTH", "EAST", "SOUTH", "WEST"};
public:
    static Direction get_next(const Direction d) {
        int index = static_cast<int>(d);
        return static_cast<Direction>((index + 1) % DIRECTIONS_NO);
    }

    static Coordinates get_move(const Direction d) {
        return direction_move[static_cast<int>(d)];
    }

    static std::string_view get_name(const Direction d) {
        return direction_name[static_cast<int>(d)];
    }
};

// Connects coordinates with direction, allows rover to move.
class Position {
private:
    Coordinates coordinates;
    Direction direction;
public:
    Position(Coordinates coordinates, Direction direction) :
        coordinates(coordinates), direction(direction) {}

    void turn_right() {
        direction = DirectionManager::get_next(direction);
    }

    void go_forward() {
        coordinates += DirectionManager::get_move(direction);
    }

    bool is_safe(std::shared_ptr<Sensor> sensor) {
        return coordinates.is_safe(std::move(sensor));
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const Position &position) {
        os << position.coordinates << " "
            << DirectionManager::get_name(position.direction);
        return os;
    }
};

// Abstract class for all actions that rover can execute.
class Action {
public:
    virtual void execute(Position &p, const sensors_t &sensors) = 0;
};

// Rover can rotate.
class Rotate : public Action {};

class RotateLeft : public Rotate {
public:
    void execute(Position &p,
                 [[maybe_unused]] const sensors_t &sensors) override {
        p.turn_right();
        p.turn_right();
        p.turn_right();
    }
};

class RotateRight : public Rotate {
public:
    void execute(Position &p,
                 [[maybe_unused]] const sensors_t &sensors) override {
        p.turn_right();
    }
};

// Rover can move.
// It is important to check safety of new field before moving.
class Move : public Action {};

class MoveForward : public Move {
public:
    void execute(Position &p, const sensors_t &sensors) override {
        Position new_position = p;
        new_position.go_forward();
        for (const auto &sensor : sensors) {
            if (!new_position.is_safe(sensor))
                throw DangerousField();
        }
        p = new_position;
    }
};

class MoveBackward : public Move {
public:
    void execute(Position &p, const sensors_t &sensors) override {
        Position new_position = p;
        new_position.turn_right();
        new_position.turn_right();
        new_position.go_forward();
        new_position.turn_right();
        new_position.turn_right();
        for (const auto &sensor : sensors) {
            if (!new_position.is_safe(sensor))
                throw DangerousField();
        }
        p = new_position;
    }
};

// Composing many moves into one.
class Compose : public Action {
private:
    std::vector<std::shared_ptr<Action>> _actions;
public:

    Compose(std::vector<std::shared_ptr<Action>> actions) :
        _actions(std::move(actions)) {}

    void execute(Position &p, const sensors_t &sensors) override {
        for (auto & _action : _actions) {
            _action->execute(p, sensors);
        }
    }
};

std::shared_ptr<MoveForward> move_forward() {
    return std::make_shared<MoveForward>();
}

std::shared_ptr<MoveBackward> move_backward() {
    return std::make_shared<MoveBackward>();
}

std::shared_ptr<RotateLeft> rotate_left() {
    return std::make_shared<RotateLeft>();
}

std::shared_ptr<RotateRight> rotate_right() {
    return std::make_shared<RotateRight>();
}

std::shared_ptr<Compose> compose(std::vector<std::shared_ptr<Action>> actions) {
    return std::make_shared<Compose>(actions);
}

using command_name_t = char;
using commands_t = std::map<command_name_t, std::shared_ptr<Action>>;

class Rover {
private:
    bool landed = false;
    bool stopped = false;
    Position position;
    commands_t commands;
    sensors_t sensors;

public:
    Rover(commands_t commands_, sensors_t sensors_) :
        // Since the rover hasn't landed yet, the position doesn't matter.
        position({0, 0}, Direction::NORTH),
        commands(std::move(commands_)),
        sensors(std::move(sensors_)) {}

    friend std::ostream& operator<<(std::ostream& os, const Rover &rover) {
        if (!rover.landed) {
            os << "unknown";
        }
        else {
            os << rover.position;
            if (rover.stopped)
                os << " stopped";
        }
        return os;
    }

    void execute(std::string command_list) {
        if (landed) {
            stopped = false;
            try {
                for (const auto &command : command_list) {
                    // Checking if command was programmed.
                    if (commands.count(command) == 0) {
                        stopped = true;
                        break;
                    }
                    // Throws exception.
                    commands[command]->execute(position, sensors);
                }
            }
            catch (DangerousField& e) {
                stopped = true;
            }
        }
        else {
            throw RoverDidNotLand();
        }
    }

    void land(const Coordinates coordinates, const Direction direction) {
        position = {coordinates, direction};
        landed = true;
        stopped = false;
    }
};

class RoverBuilder {
private:
    commands_t commands;
    sensors_t sensors;
public:
    RoverBuilder& program_command(command_name_t name,
                                  std::shared_ptr<Action> action) {
        commands[name] = std::move(action);
        return *this;
    }

    RoverBuilder& add_sensor(std::unique_ptr<Sensor> sensor) {
        sensors.push_back(std::move(sensor));
        return *this;
    }

    Rover build() {
        return {commands, std::move(sensors)};
    }
};


#endif //ROVER_H
